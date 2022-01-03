/*
Copyright 2021 Siyuan Pan <pansiyuan.cs@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#include <Ajisai/Core/Camera.h>
#include <Ajisai/Core/Film.h>
#include <Ajisai/Core/Mesh.h>
#include <Ajisai/Integrators/Integrator.h>
#include <Ajisai/Math/Math.h>
#include <Ajisai/PluginManager/AbstractManager.h>

#include <Ajisai/Core/Parallel.h>
#include <Ajisai/Core/Random.h>

#include <Ajisai/Integrators/BidirectionalPath.h>
#include <Ajisai/Util/Distribution.h>

#include <array>
#include <cassert>

namespace Ajisai::Integrators {

using namespace Ajisai::Core;
using namespace Ajisai::Math;
using namespace Ajisai::Util;

static float ErfInv(float x) {
  float w, p;

  x = std::clamp(x, -0.99999f, 0.99999f);
  w = -std::log((1 - x) * (1 + x));
  if (w < 5) {
    w = w - 2.5f;
    p = 2.81022636e-08f;
    p = 3.43273939e-07f + p * w;
    p = -3.5233877e-06f + p * w;
    p = -4.39150654e-06f + p * w;
    p = 0.00021858087f + p * w;
    p = -0.00125372503f + p * w;
    p = -0.00417768164f + p * w;
    p = 0.246640727f + p * w;
    p = 1.50140941f + p * w;
  } else {
    w = std::sqrt(w) - 3;
    p = -0.000200214257f;
    p = 0.000100950558f + p * w;
    p = 0.00134934322f + p * w;
    p = -0.00367342844f + p * w;
    p = 0.00573950773f + p * w;
    p = -0.0076224613f + p * w;
    p = 0.00943887047f + p * w;
    p = 1.00167406f + p * w;
    p = 2.83297682f + p * w;
  }
  return p * x;
}

static constexpr int pow = 1;

template <int pow>
float MIS(float fVal) {
  // Use power heuristic
  return std::pow(fVal, pow);
}

class MetropolisSampler : public Sampler {
 private:
  struct PrimarySample {
    float value = 0.f;
    uint64_t modifiedIteration = 0u;

    float backupValue = 0.f;
    uint64_t backupModifiedIteration = 0u;

    void Backup() {
      backupValue = value;
      backupModifiedIteration = modifiedIteration;
    }

    void Restore() {
      value = backupValue;
      modifiedIteration = backupModifiedIteration;
    }
  };

  PCG32 rng;
  const float sigma;
  const float largeStepProb;
  const int streamCount = 3;
  int sampleIndex;
  int streamIndex;
  uint64_t prevLargeStepIteration = 0u;
  uint64_t currentIteration = 0u;
  bool largeStep = true;

  std::vector<PrimarySample> samples;

 public:
  MetropolisSampler(const float sigma, const float largeStepProb, uint64_t seed)
      : sigma(sigma), largeStepProb(largeStepProb), rng(seed) {}
  virtual ~MetropolisSampler() {}
  virtual void SetSeed(std::size_t seed) override { rng = PCG32(seed); }
  virtual std::shared_ptr<Sampler> Copy() const override {
    return std::make_shared<MetropolisSampler>(*this);
  }
  virtual float Next1D() override {
    const int index = GetNextIndex();
    Mutate(index);

    return samples[index].value;
  }
  virtual Math::Vector2f Next2D() override { return {Next1D(), Next1D()}; }

  void StartStream(const int index) {
    assert(index < streamCount);
    streamIndex = index;
    sampleIndex = 0;
  }

  void Mutate(const int index) {
    if (index >= samples.size()) samples.resize(index + 1);

    auto& sample = samples[index];
    if (sample.modifiedIteration < prevLargeStepIteration) {
      sample.value = rng.next_float();
      sample.modifiedIteration = prevLargeStepIteration;
    }

    sample.Backup();
    if (largeStep)
      sample.value = rng.next_float();
    else {
      int numSmallSteps = currentIteration - sample.modifiedIteration;

      const float Sqrt2 = 1.41421356237309504880f;
      float normalSample = Sqrt2 * ErfInv(2 * rng.next_float() - 1);

      float effSigma = sigma * std::sqrt(float(numSmallSteps));
      sample.value += normalSample * effSigma;
      sample.value -= std::floor(sample.value);
    }

    sample.modifiedIteration = currentIteration;
  }

  void Accept() {
    if (largeStep) prevLargeStepIteration = currentIteration;
  }

  void Reject() {
    for (auto& it : samples) {
      if (it.modifiedIteration == currentIteration) it.Restore();
    }
    currentIteration--;
  }

  void StartIteration() {
    largeStep = rng.next_float() < largeStepProb;
    currentIteration++;
  }

 private:
  int GetNextIndex() { return streamIndex + streamCount * sampleIndex++; }
};

class MMLTIntegrator : public Integrator {
 public:
  explicit MMLTIntegrator(PluginManager::AbstractManager& manager,
                          const std::string plugin)
      : Integrator{manager, plugin} {}

  virtual Spectrum Li(Scene* scene, Camera* camera, const Vector2i& raster,
                      Sampler* sampler) const override {
    return {};
  }

  virtual void Render(Scene* scene, Camera* camera,
                      Sampler* pSampler) const override {
    constexpr const size_t numBootstrapSample = numBootstrap * (maxDepth + 1);
    std::array<float, numBootstrapSample> boostrapWeights{0.f};

    parallel_for(
        numBootstrap,
        [&](uint32_t i, uint32_t tid) {
          PCG32 rng(i);

          for (int depth = 0; depth <= maxDepth; ++depth) {
            uint64_t seed = depth + i * (maxDepth + 1);
            MetropolisSampler sampler(sigma, largeStepProb, seed);

            Vector2f rasterPos;
            boostrapWeights[seed] =
                EvalSample(scene, &sampler, camera, depth, &rasterPos, rng)
                    .removeNaN()
                    .luminance();
            // printf("%f\n", boostrapWeights[seed]);
          }
        },
        1024);

    Distribution bootstrapDist(boostrapWeights.data(), boostrapWeights.size());
    float b = bootstrapDist.funcInt * (maxDepth + 1);
    printf("b %f Int %f\n", b, bootstrapDist.funcInt);

    uint64_t numTotalMutations = spp * camera->GetPixelCount();
    uint64_t totalSamples;

    parallel_for(
        numChains,
        [&](uint32_t i, uint32_t tid) {
          uint64_t numChainMutations =
              std::min((i + 1) * numTotalMutations / numChains,
                       numTotalMutations) -
              i * numTotalMutations / numChains;

          // printf("%d\n", numChainMutations);

          PCG32 rng(i);

          int bootstrapIndex = bootstrapDist.sampleDiscrete(rng.next_float());
          int depth = bootstrapIndex % (maxDepth + 1);
          // printf("%d %d\n", bootstrapIndex, depth);

          MetropolisSampler sampler(sigma, largeStepProb, bootstrapIndex);

          Vector2f currentRaster;
          auto currentLum =
              EvalSample(scene, &sampler, camera, depth, &currentRaster, rng);

          // Run the Markov chain for numChainMutations steps
          for (uint64_t j = 0; j < numChainMutations; ++j) {
            sampler.StartIteration();

            Vector2f proposedRaster;
            auto proposedLum = EvalSample(scene, &sampler, camera, depth,
                                          &proposedRaster, rng);

            // printf("%f %f %f %f %f %d\n", proposedRaster[0],
            // proposedRaster[1],
            //        proposedLum[0], proposedLum[1], proposedLum[2], depth);
            float acceptProb =
                std::min(1.0f, proposedLum.luminance() /
                                   (currentLum.luminance() + 1e-4f));
            if (acceptProb > 0.f) {
              camera->GetFilm()->AddSplat(
                  proposedLum * acceptProb / (proposedLum.luminance() + 1e-4f),
                  proposedRaster);

              // auto tmp =
              //     proposedLum * acceptProb / (proposedLum.luminance() +
              //     1e-4f);
              // printf("%f %f %f\n", tmp[0], tmp[1], tmp[2]);
            }

            camera->GetFilm()->AddSplat(currentLum * (1 - acceptProb) /
                                            (currentLum.luminance() + 1e-4f),
                                        currentRaster);

            if (rng.next_float() < acceptProb) {
              currentRaster = proposedRaster;
              currentLum = proposedLum;
              sampler.Accept();
            } else {
              sampler.Reject();
            }
          }
        },
        1024);

    camera->GetFilm()->ScaleToPixel(b / spp);
    printf("%f\n", b / spp);
  }

  Spectrum EvalSample(Scene* scene, MetropolisSampler* pSampler, Camera* camera,
                      const int connectDepth, Vector2f* pRasterPos,
                      PCG32& rng) const {
    pSampler->StartStream(0);

    int lightLen, eyeLen, numStrategies;
    if (connectDepth == 0) {
      numStrategies = 1;
      lightLen = 0;
      eyeLen = 2;
    } else {
      numStrategies = connectDepth + 2;
      lightLen = std::min(int(pSampler->Next1D() * float(numStrategies)),
                          numStrategies - 1);
      eyeLen = numStrategies - lightLen;
    }

    BDPTIntegrator::Vertex* lightPath = (BDPTIntegrator::Vertex*)calloc(
        lightLen, sizeof(BDPTIntegrator::Vertex));
    int numLightVertex;
    if (BDPTIntegrator::GenerateLightPath(scene, pSampler, lightLen, lightPath,
                                          camera, &numLightVertex, -1,
                                          false) != lightLen) {
      free((void*)lightPath);
      return {0.f, 0.f, 0.f};
    }

    pSampler->StartStream(1);

    *pRasterPos = pSampler->Next2D() *
                  Vector2f{(float)camera->GetFilm()->Dimension().x(),
                           (float)camera->GetFilm()->Dimension().y()};
    // printf("%f %f\n", pRasterPos->x(), pRasterPos->y());
    auto ray = camera->GenerateRay(*pRasterPos);
    // printf("%f %f %f %f %f %f\n", ray.o[0], ray.o[1], ray.o[2], ray.d[0],
    //        ray.d[1], ray.d[2]);

    BDPTIntegrator::PathState cameraPathState;
    BDPTIntegrator::SampleCamera(camera, ray, pSampler, cameraPathState);

    // Trace camera sub-path
    Intersection isect;
    unsigned int specular;
    SurfaceInteraction si;
    while (eyeLen > 1) {
      Ray pathRay(cameraPathState.origin, cameraPathState.direction);
      // Intersection isect;
      if (!scene->Intersect(pathRay, &isect)) {
        break;
      }

      float cosIn = std::abs(dot(isect.Ng, -pathRay.d));
      cameraPathState.DVCM *= MIS<pow>(isect.t * isect.t);
      cameraPathState.DVCM /= MIS<pow>(cosIn);
      cameraPathState.DVC /= MIS<pow>(cosIn);

      if (++cameraPathState.PathLength >= eyeLen) {
        break;
      }

      Triangle triangle{};
      isect.mesh->GetTriangle(isect.triId, &triangle);
      auto p = pathRay.Point(isect.t);
      // SurfaceInteraction si(-pathRay.d, p, triangle, isect);
      si = SurfaceInteraction(-pathRay.d, p, triangle, isect);
      isect.mesh->GetMaterial()->ComputeScatteringFunction(&si);

      BSDFSamplingRecord bRec(si, pSampler->Next2D());
      si.bsdf->Sample(bRec);
      if (bRec.pdf <= 0.f) break;

      Spectrum f = bRec.f;
      Vector3f wi = bRec.wi;
      float scatteredPdf = bRec.pdf;

      float revPdf = si.bsdf->EvaluatePdf(wi, si.bsdf->toLocal(-pathRay.d));

      if (f.isBlack() || scatteredPdf == 0.f) break;

      cameraPathState.origin = si.p;
      cameraPathState.direction = si.bsdf->toWorld(bRec.wi);

      float cosOut = std::abs(dot(si.Ns, si.bsdf->toWorld(bRec.wi)));
      specular = bRec.type & BxDFType::BSDF_SPECULAR;
      if (!specular) {
        cameraPathState.SpecularPath &= 0;

        cameraPathState.DVCM =
            MIS<pow>(cosOut / scatteredPdf) *
            (cameraPathState.DVC * MIS<pow>(revPdf) + cameraPathState.DVCM);
        cameraPathState.DVC = MIS<pow>(1.0f / scatteredPdf);
      } else {
        cameraPathState.SpecularPath &= 1;

        cameraPathState.DVCM = 0.f;
        cameraPathState.DVC *= MIS<pow>(cosOut);
      }

      cameraPathState.throughput *= f * cosOut / scatteredPdf;
    }

    Spectrum ret;

    if (cameraPathState.PathLength == eyeLen) {
      pSampler->StartStream(2);

      if (lightLen == 0) {
        if (isect.mesh->IsEmitter()) {
          auto ray = Ray(cameraPathState.origin, cameraPathState.direction);
          // auto tmp = BDPTIntegrator::HittingLightSource(
          //     scene, ray, isect, isect.mesh->GetLight(isect.triId).get(),
          //     cameraPathState);
          ret = cameraPathState.throughput *
                BDPTIntegrator::HittingLightSource(
                    scene, ray, isect, isect.mesh->GetLight(isect.triId).get(),
                    cameraPathState);
        }
      } else if (eyeLen == 1) {
        if (numLightVertex > 0) {
          BDPTIntegrator::Vertex& lightVertex = lightPath[numLightVertex - 1];

          if (lightVertex.length == lightLen) {
            Vector2f ptRaster;
            ret = BDPTIntegrator::ConnectToCamera(
                scene, camera, pSampler, lightVertex.si, lightVertex, ptRaster);

            *pRasterPos = Vector2f(ptRaster.x(), ptRaster.y());
          }
        }
      } else if (lightLen == 1) {
        if (si.bsdf != nullptr && !specular) {
          auto ray = Ray(cameraPathState.origin, cameraPathState.direction);
          // printf("%f %f %f %f %f %f\n", ray.o[0], ray.o[1], ray.o[2],
          // ray.d[0],
          //        ray.d[1], ray.d[2]);
          // printf("%f %f %f %f %f %f\n", si.p[0], si.p[1], si.p[2], si.Ng[0],
          //        si.Ng[1], si.Ng[2]);
          ret = cameraPathState.throughput *
                BDPTIntegrator::ConnectToLight(scene, ray, si, pSampler,
                                               cameraPathState);
        }
      } else {
        if (numLightVertex > 0 && si.bsdf != nullptr && !specular) {
          const BDPTIntegrator::Vertex& lightVertex =
              lightPath[numLightVertex - 1];

          ret = lightVertex.throughput * cameraPathState.throughput *
                BDPTIntegrator::ConnectVertex(scene, si, lightVertex,
                                              cameraPathState);
        }
      }

      ret *= numStrategies;
    }

    free((void*)lightPath);

    return ret;
  }

 private:
  static constexpr int numBootstrap = (1 << 15);  // 100000;  //(1 << 17);
  static constexpr int maxDepth = 8;              // 8;
  static constexpr float sigma = 0.01f;
  static constexpr float largeStepProb = 0.3f;
  static constexpr int spp = 1024;       // 16;         // 4096;
  static constexpr int numChains = 512;  // 100;  // 2048;
};

}  // namespace Ajisai::Integrators

AJISAI_PLUGIN_REGISTER(MMLTIntegrator, Ajisai::Integrators::MMLTIntegrator,
                       "ajisai.integrators.Integrator/0.0.1")
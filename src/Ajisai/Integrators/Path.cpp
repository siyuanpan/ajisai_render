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

#include <future>

namespace Ajisai::Integrators {

using namespace Ajisai::Core;
using namespace Ajisai::Math;

// Multiple Importance Sampling
static float MisWeight(float pdfA, float pdfB, float beta = 2.0f) {
  pdfA = std::pow(pdfA, beta);
  pdfB = std::pow(pdfB, beta);
  return pdfA / (pdfA + pdfB);
}

class PathIntegrator : public Integrator {
 public:
  explicit PathIntegrator(PluginManager::AbstractManager& manager,
                          const std::string plugin)
      : Integrator{manager, plugin} {}

  virtual Math::Spectrum Li(Core::Scene* scene, Core::Camera* camera,
                            const Math::Vector2i& raster,
                            Core::Sampler* sampler) const override {
    // const float u =
    //     (raster.x() + sampler->Next1D()) /
    //     camera->GetFilm()->Dimension().x();
    // const float v =
    //     (raster.y() + sampler->Next1D()) /
    //     camera->GetFilm()->Dimension().y();

    auto ray =
        camera->GenerateRay(sampler->Next2D(), sampler->Next2D(), raster);

    Math::Spectrum L(0), pathThroughput(1);
    bool specularBounce = true;
    Intersection prevIts;
    float prevPdf;
    // auto pathRay = ray;
    for (auto bounce = 0;; bounce++) {
      Intersection intersection;
      // DifferentialGeom diffGeom{};
      auto intersected = scene->Intersect(ray, &intersection);
      // auto intersected = scene->Intersect(ray, &diffGeom);

      if (pathThroughput.isBlack()) break;

      // scene->PostIntersect(ray, &diffGeom);

      if (specularBounce) {
        if (intersected) {
          L += pathThroughput * intersection.Le(-ray.d);
          // L += pathThroughput * diffGeom.emit(-ray.d);
        }
      } else {
        if (intersected && intersection.mesh->IsEmitter()) {
          auto light = intersection.mesh->GetLight(intersection.triId);
          auto lightPdf =
              light->pdfLi(prevIts, ray.d) * scene->PdfLight(light.get());
          if (lightPdf != 0)
            L += pathThroughput * intersection.Le(-ray.d) *
                 MisWeight(prevPdf, lightPdf);
        }
      }

      if (!intersected || bounce >= maxDepth) break;

      Triangle triangle{};
      intersection.mesh->GetTriangle(intersection.triId, &triangle);
      auto p = ray.Point(intersection.t);
      SurfaceInteraction si(-ray.d, p, triangle, intersection);
      intersection.mesh->GetMaterial()->ComputeScatteringFunction(&si);

      BSDFSamplingRecord bRec(si, sampler->Next2D());
      si.bsdf->Sample(bRec);
      if (bRec.pdf <= 0.f) break;

      specularBounce = bRec.type & BxDFType::BSDF_SPECULAR;
      if (!specularBounce) {
        float lightPdf = 0.f;
        auto sampleLight = scene->SampleOneLight(sampler->Next1D(), &lightPdf);
        if (sampleLight && lightPdf > 0) {
          LightSamplingRecord lRec;
          sampleLight->SampleLi(sampler->Next2D(), p, lRec);
          lightPdf *= lRec.pdf;
          auto wi = si.bsdf->toLocal(lRec.wi);
          auto wo = si.bsdf->toLocal(si.wo);
          auto f =
              si.bsdf->Evaluate(wo, wi) * std::abs(Math::dot(lRec.wi, si.Ns));
          Intersection shadowIntersection;
          if (lightPdf > 0 &&
              !scene->Intersect(lRec.shadowRay, &shadowIntersection)) {
            auto scatteringPdf = si.bsdf->EvaluatePdf(wo, wi);
            L += pathThroughput * f * lRec.Li / lightPdf *
                 MisWeight(lightPdf, scatteringPdf);
          }
        }
      }

      auto wi = si.bsdf->toWorld(bRec.wi);
      if (bRec.f.isBlack() || bRec.pdf <= 0.f) break;
      pathThroughput *= bRec.f * std::abs(Math::dot(wi, si.Ns)) / bRec.pdf;
      ray = si.SpawnRay(wi);
      prevIts = intersection;
      prevPdf = bRec.pdf;
      if (bounce >= rrDepth) {
        float q = std::min(0.95f, pathThroughput.max());
        if (sampler->Next1D() >= q) break;
        pathThroughput /= q;
      }
    }

    return L;
  }

  virtual void Render(Core::Scene* scene, Core::Camera* camera,
                      Core::Sampler* sampler) const {}

 private:
  int spp = 16;
  int rrDepth = 5, maxDepth = 16;
};

}  // namespace Ajisai::Integrators

AJISAI_PLUGIN_REGISTER(PathIntegrator, Ajisai::Integrators::PathIntegrator,
                       "ajisai.integrators.Integrator/0.1.1")
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
    const float u =
        (raster.x() + sampler->Next1D()) / camera->GetFilm()->Dimension().x();
    const float v =
        (raster.y() + sampler->Next1D()) / camera->GetFilm()->Dimension().y();
    // auto ray1 = camera->GenerateRay(u, v);
    // printf("old (%f %f %f) (%f %f %f)\n", ray.o[0], ray.o[1], ray.o[2],
    //        ray.d[0], ray.d[1], ray.d[2]);
    auto ray =
        camera->GenerateRay(sampler->Next2D(), sampler->Next2D(), raster);
    // printf("old (%f %f %f) (%f %f %f) new (%f %f %f) (%f %f %f)\n",
    // ray1.o[0],
    //        ray1.o[1], ray1.o[2], ray1.d[0], ray1.d[1], ray1.d[2], ray.o[0],
    //        ray.o[1], ray.o[2], ray.d[0], ray.d[1], ray.d[2]);
    // ray.o = ray1.o;
    // ray.d = ray1.d;
    // printf("(%f %f %f) %f (%f %f %f) %f\n", ray1.d[0], ray1.d[1], ray1.d[2],
    //        Math::dot(ray1.d, ray.d), ray.d[0], ray.d[1], ray.d[2],
    //        ray.d.length());

    Math::Spectrum L(0), pathThroughput(1);
    bool specularBounce = true;
    Intersection prevIts;
    float prevPdf;
    // auto pathRay = ray;
    for (auto bounce = 0;; bounce++) {
      Intersection intersection;
      auto intersected = scene->Intersect(ray, &intersection);

      if (pathThroughput.isBlack()) break;

      if (specularBounce) {
        if (intersected) {
          L += pathThroughput * intersection.Le(-ray.d);
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

      specularBounce = bRec.type & BSDFType::BSDF_SPECULAR;
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
    // Math::Spectrum Li(0), beta(1);
    // bool specular = false;
    // Intersection prevIts;
    // float prevPdf;
    // for (int depth = 0; depth < maxDepth; ++depth) {
    //   Intersection intersection;
    //   if (scene->Intersect(ray, &intersection)) {
    //     auto& mesh = scene->GetMesh(intersection.meshId);
    //     auto material = mesh.GetMaterial();
    //     // auto mesh = intersection.mesh;
    //     if (mesh.IsEmitter()) {
    //       if (depth == 0) {
    //         Li += beta * mesh.Le(-ray.d);
    //       } else {
    //         auto light = mesh.GetLight(intersection.triId);
    //         auto lightPdf =
    //             light->pdfLi(prevIts, ray.d) * scene->PdfLight(light.get());
    //         // std::cout << "prevPdf : " << prevPdf << std::endl;
    //         if (lightPdf != 0)
    //           // std::cout << "lightPdf : " << lightPdf << std::endl;
    //           Li += beta * mesh.Le(-ray.d) * MisWeight(prevPdf, lightPdf);
    //       }
    //     }
    //     Triangle triangle{};
    //     mesh.GetTriangle(intersection.triId, &triangle);
    //     auto p = ray.Point(intersection.t);
    //     SurfaceInteraction event(-ray.d, p, triangle, intersection);
    //     material->ComputeScatteringFunction(&event);

    //     BSDFSamplingRecord bRec(event, sampler->Next2D());
    //     event.bsdf->Sample(bRec);
    //     if (bRec.pdf <= 0) break;

    //     // light sample
    //     specular = bRec.type & BSDFType::BSDF_SPECULAR;
    //     float lightPdf = 0.f;
    //     auto sampleLight = scene->SampleOneLight(sampler->Next1D(),
    //     &lightPdf); if (sampleLight && lightPdf > 0) {
    //       LightSamplingRecord lRec;
    //       sampleLight->SampleLi(sampler->Next2D(), p, lRec);
    //       lightPdf *= lRec.pdf;
    //       auto wi = event.bsdf->toLocal(lRec.wi);
    //       auto wo = event.bsdf->toLocal(event.wo);
    //       auto f = event.bsdf->Evaluate(wo, wi) *
    //                std::abs(Math::dot(lRec.wi, event.Ns));
    //       Intersection shadowIntersection;
    //       if (lightPdf > 0 &&
    //           !scene->Intersect(lRec.shadowRay, &shadowIntersection)) {
    //         if (specular) {
    //           Li += beta * f * lRec.Li / lightPdf;
    //         } else {
    //           auto scatteringPdf = event.bsdf->EvaluatePdf(wo, wi);
    //           Li += beta * f * lRec.Li / lightPdf *
    //                 MisWeight(lightPdf, scatteringPdf);
    //         }
    //       }
    //     }

    //     auto wi = event.bsdf->toWorld(bRec.wi);
    //     beta *= bRec.f * std::abs(Math::dot(wi, event.Ns)) / bRec.pdf;
    //     if (depth >= rrDepth) {
    //       float q = std::min(0.95f, beta.max());
    //       if (sampler->Next1D() >= q) break;
    //       beta /= q;
    //     }
    //     ray = event.SpawnRay(wi);
    //     prevIts = intersection;
    //     prevPdf = bRec.pdf;
    //   } else {
    //     Li += beta * Math::Spectrum(0);
    //     break;
    //   }
    // }
    // return Li;
  }

 private:
  int spp = 16;
  int rrDepth = 5, maxDepth = 16;
};

}  // namespace Ajisai::Integrators

AJISAI_PLUGIN_REGISTER(PathIntegrator, Ajisai::Integrators::PathIntegrator,
                       "ajisai.integrators.Integrator/0.1.1")
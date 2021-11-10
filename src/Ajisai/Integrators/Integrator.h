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

#ifndef AJISAI_INTEGRATORS_INTEGRATOR_H_
#define AJISAI_INTEGRATORS_INTEGRATOR_H_

// #include <future>
// #include <mutex>

// #include "Ajisai/Core/Camera.h"
#include <Ajisai/Core/Film.h>
// #include "Ajisai/Core/Mesh.h"
// #include "Ajisai/Core/Parallel.hpp"
// #include "Ajisai/Core/Sampler.h"
#include <Ajisai/Core/Scene.h>
#include <Ajisai/Math/Math.h>

namespace Ajisai::Integrators {

struct RenderContext {
  std::shared_ptr<Ajisai::Core::Film> film;
  std::shared_ptr<Ajisai::Core::Camera> camera;
  std::shared_ptr<Ajisai::Core::Scene> scene;
  std::shared_ptr<Ajisai::Core::Sampler> sampler;
};

class RenderTask {
 public:
  // explicit RenderTask(const RenderContext& ctx, int spp) : ctx(ctx), spp(spp)
  // {}

  virtual ~RenderTask() {}

  // void Wait() { future.wait(); }
  virtual void Wait() = 0;

  virtual Math::Spectrum Li(Core::Ray ray, Core::Sampler* sampler) = 0;

  // auto Li(Ray ray, Sampler* sampler) {
  //   auto scene = ctx.scene;
  //   Math::Spectrum Li(0), beta(1);
  //   for (int depth = 0; depth < 5; ++depth) {
  //     Intersection intersection;
  //     if (scene->Intersect(ray, &intersection)) {
  //       auto& mesh = scene->GetMesh(intersection.meshId);
  //       Triangle triangle{};
  //       mesh.GetTriangle(intersection.triId, &triangle);
  //       auto p = ray.Point(intersection.t);
  //       ScatteringEvent event(-ray.d, p, triangle, intersection);
  //       mesh.computeScatteringFunctions(&event);
  //       BSDFSamplingRecord bRec(event, sampler->Next2D());
  //       event.bsdf->Sample(bRec);

  //       auto wi = event.bsdf->toWorld(bRec.wi);
  //       beta *= bRec.f * std::abs(Math::dot(wi, event.Ns)) / bRec.pdf;
  //       ray = event.SpawnRay(wi);
  //     } else {
  //       Li += beta * Math::Spectrum(1);
  //       break;
  //     }
  //   }
  //   return Li;
  // }

  virtual void Start() = 0;

  // void Start() {
  //   future = std::async(std::launch::async, [=]() {
  //     auto beginTime = std::chrono::high_resolution_clock::now();
  //     auto film = ctx.film;
  //     auto camera = ctx.camera;
  //     auto scene = ctx.scene;
  //     auto& _sampler = ctx.sampler;
  //     auto nTiles = (film->Dimension() + Math::Vector2i(TileSize - 1)) /
  //                   Math::Vector2i(TileSize);
  //     parallel_for_2D(nTiles, [=](Math::Vector2i tilePos, uint32_t tid) {
  //       (void)tid;
  //       Math::Bounds2i tileBounds =
  //           Math::Bounds2i{tilePos * (int)TileSize,
  //                          (tilePos + Math::Vector2i(1)) * (int)TileSize};
  //       auto tile = film->GetTile(tileBounds);
  //       auto sampler = _sampler->Copy();
  //       for (int y = tile.bounds.min().y(); y < tile.bounds.max().y(); ++y) {
  //         for (int x = tile.bounds.min().x(); x < tile.bounds.max().x(); ++x)
  //         {
  //           sampler->SetSeed(x + y * film->Dimension().x());
  //           for (int s = 0; s < spp; ++s) {
  //             const float u = (x + sampler->Next1D()) /
  //             film->Dimension().x(); const float v = (y + sampler->Next1D())
  //             / film->Dimension().y(); auto ray = camera->GenerateRay(u, v);
  //             auto Li = this->Li(ray, sampler.get());
  //             tile.AddSample(Math::Vector2i{x, y}, Li, 1.0f);
  //           }
  //         }
  //       }
  //       std::lock_guard<std::mutex> lk(mutex);
  //       film->MergeTile(tile);
  //     });
  //     auto endTime = std::chrono::high_resolution_clock::now();
  //     std::chrono::duration<double> elapsed = (endTime - beginTime);
  //     std::cout << "Rendering done in " << elapsed.count() << "  secs\n";
  //   });
  // }

  // std::shared_ptr<const Film> GetFilm() { return ctx.film; }
  virtual std::shared_ptr<const Core::Film> GetFilm() = 0;

  // std::future<void> future;
  // RenderContext ctx;
  // std::mutex mutex;
  // int spp;
};

class Integrator {
 public:
  // int spp = 16;
  virtual std::shared_ptr<RenderTask> CreateRenderTask(
      const RenderContext& ctx) = 0;
  // {
  //   return std::make_shared<RenderTask>(ctx, spp);
  // }
};

class PathIntegrator : public Integrator {
 public:
  virtual std::shared_ptr<RenderTask> CreateRenderTask(
      const RenderContext& ctx) override;

 private:
  int spp = 16;
  int minDepth = 5, maxDepth = 16;
};
}  // namespace Ajisai::Integrators
#endif
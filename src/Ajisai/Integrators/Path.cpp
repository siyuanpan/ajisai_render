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

class PTRenderTask : public RenderTask {
 private:
  std::future<void> future;
  RenderContext ctx;
  std::mutex mutex;
  int spp;
  int minDepth;
  int maxDepth;

 public:
  PTRenderTask(const RenderContext& ctx, int spp, int minDepth, int maxDepth)
      : ctx(ctx), spp(spp), minDepth(minDepth), maxDepth(maxDepth) {}

  virtual ~PTRenderTask() {}

  virtual std::shared_ptr<const Film> GetFilm() { return ctx.film; }

  virtual void Wait() { future.wait(); }

  virtual Math::Spectrum Li(Ray ray, Sampler* sampler) {
    auto scene = ctx.scene;
    Math::Spectrum Li(0), beta(1);
    for (int depth = 0; depth < 5; ++depth) {
      Intersection intersection;
      if (scene->Intersect(ray, &intersection)) {
        auto& mesh = scene->GetMesh(intersection.meshId);
        Triangle triangle{};
        mesh.GetTriangle(intersection.triId, &triangle);
        auto p = ray.Point(intersection.t);
        ScatteringEvent event(-ray.d, p, triangle, intersection);
        mesh.computeScatteringFunctions(&event);
        BSDFSamplingRecord bRec(event, sampler->Next2D());
        event.bsdf->Sample(bRec);

        auto wi = event.bsdf->toWorld(bRec.wi);
        beta *= bRec.f * std::abs(Math::dot(wi, event.Ns)) / bRec.pdf;
        ray = event.SpawnRay(wi);
      } else {
        Li += beta * Math::Spectrum(1);
        break;
      }
    }
    return Li;
  }

  virtual void Start() {
    future = std::async(std::launch::async, [=]() {
      auto beginTime = std::chrono::high_resolution_clock::now();

      auto film = ctx.film;
      auto camera = ctx.camera;
      auto scene = ctx.scene;
      auto& _sampler = ctx.sampler;
      auto nTiles = (film->Dimension() + Math::Vector2i(TileSize - 1)) /
                    Math::Vector2i(TileSize);
      parallel_for_2D(nTiles, [=](Math::Vector2i tilePos, uint32_t tid) {
        (void)tid;
        Math::Bounds2i tileBounds =
            Math::Bounds2i{tilePos * (int)TileSize,
                           (tilePos + Math::Vector2i(1)) * (int)TileSize};
        auto tile = film->GetTile(tileBounds);
        auto sampler = _sampler->Copy();
        for (int y = tile.bounds.min().y(); y < tile.bounds.max().y(); ++y) {
          for (int x = tile.bounds.min().x(); x < tile.bounds.max().x(); ++x) {
            sampler->SetSeed(x + y * film->Dimension().x());
            for (int s = 0; s < spp; ++s) {
              const float u = (x + sampler->Next1D()) / film->Dimension().x();
              const float v = (y + sampler->Next1D()) / film->Dimension().y();
              auto ray = camera->GenerateRay(u, v);
              auto Li = this->Li(ray, sampler.get());
              tile.AddSample(Math::Vector2i{x, y}, Li, 1.0f);
            }
          }
        }
        std::lock_guard<std::mutex> lk(mutex);
        film->MergeTile(tile);
      });
      auto endTime = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed = (endTime - beginTime);
      std::cout << "Rendering done in " << elapsed.count() << "  secs\n";
    });
  }
};

// std::shared_ptr<RenderTask> PathIntegrator::CreateRenderTask(
//     const RenderContext& ctx) {
//   return std::make_shared<PTRenderTask>(ctx, spp, minDepth, maxDepth);
// }

class PathIntegrator : public Integrator {
 public:
  //   explicit PathIntegrator() {}
  explicit PathIntegrator(PluginManager::AbstractManager& manager,
                          const std::string plugin)
      : Integrator{manager, plugin} {}

  virtual std::shared_ptr<RenderTask> CreateRenderTask(
      const RenderContext& ctx) override {
    std::cout << "cp1\n";
    return std::make_shared<PTRenderTask>(ctx, spp, minDepth, maxDepth);
    std::cout << "cp5\n";
  }

 private:
  int spp = 16;
  int minDepth = 5, maxDepth = 16;
};

}  // namespace Ajisai::Integrators

AJISAI_PLUGIN_REGISTER(PathIntegrator, Ajisai::Integrators::PathIntegrator,
                       "ajisai.integrators.Integrator/0.0.1")
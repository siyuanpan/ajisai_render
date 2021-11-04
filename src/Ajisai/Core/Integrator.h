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

#ifndef AJISAI_CORE_INTEGRATOR_H_
#define AJISAI_CORE_INTEGRATOR_H_

#include <future>
#include <mutex>

#include "Ajisai/Core/Camera.h"
#include "Ajisai/Core/Film.h"
#include "Ajisai/Core/Mesh.h"
#include "Ajisai/Core/Parallel.hpp"
#include "Ajisai/Core/Sampler.h"
#include "Ajisai/Core/Scene.h"
#include "Ajisai/Math/Math.h"

namespace Ajisai::Core {

struct RenderContext {
  std::shared_ptr<Film> film;
  std::shared_ptr<Camera> camera;
  std::shared_ptr<Scene> scene;
  std::shared_ptr<Sampler> sampler;
};

class RenderTask {
 public:
  explicit RenderTask(const RenderContext& ctx) : ctx(ctx) {}

  void Wait() { future.wait(); }

  void Start() {
    future = std::async(std::launch::async, [=]() {
      auto film = ctx.film;
      auto camera = ctx.camera;
      auto scene = ctx.scene;
      auto& sampler = ctx.sampler;
      auto nTiles = (film->Dimension() + Math::Vector2i(TileSize - 1)) /
                    Math::Vector2i(TileSize);
      parallel_for_2D(nTiles, [=](Math::Vector2i tilePos, uint32_t tid) {
        (void)tid;
        Math::Bounds2i tileBounds =
            Math::Bounds2i{tilePos * (int)TileSize,
                           (tilePos + Math::Vector2i(1)) * (int)TileSize};
        auto tile = film->GetTile(tileBounds);
        for (int y = tile.bounds.min().y(); y < tile.bounds.max().y(); ++y) {
          for (int x = tile.bounds.min().x(); x < tile.bounds.max().x(); ++x) {
            const float u = (x + 0.5f) / film->Dimension().x();
            const float v = (y + 0.5f) / film->Dimension().y();
            // Intersection intersection;
            auto ray = camera->GenerateRay(u, v);
            Math::Spectrum Li(0), beta(1);
            // if (scene->Intersect(ray, &intersection)) {
            //   tile.AddSample(
            //       Math::Vector2i{x, y},
            //       Math::Spectrum{intersection.Ng[0], intersection.Ng[1],
            //                      intersection.Ng[2]},
            //       1.0f);
            // } else
            //   tile.AddSample(Math::Vector2i{x, y}, Math::Spectrum(0), 1.f);
            for (int depth = 0; depth < 5; ++depth) {
              Intersection intersection;
              if (scene->Intersect(ray, &intersection)) {
                auto& mesh = scene->GetMesh(intersection.meshId);
                // if (mesh.IsEmitter()) {
                //   Li +=
                // }
                Triangle triangle{};
                mesh.GetTriangle(intersection.triId, &triangle);
                auto p = ray.Point(intersection.t);
                ScatteringEvent event(-ray.d, p, triangle, intersection);
                mesh.computeScatteringFunctions(&event);
                BSDFSamplingRecord bRec(event, sampler->Next2D());
                event.bsdf->Sample(bRec);
                // std::cout << "pdf: " << bRec.pdf << std::endl;

                // std::cout << "wi: " << bRec.wi[0] << " " << bRec.wi[1] << " "
                //           << bRec.wi[2] << std::endl;
                auto wi = event.bsdf->toWorld(bRec.wi);
                beta *= bRec.f * std::abs(Math::dot(wi, event.Ns)) / bRec.pdf;
                ray = event.SpawnRay(wi);
              } else {
                Li += beta * Math::Spectrum(1);
                break;
              }

              // if (depth == 4) {
              //   Li += beta * Math::Spectrum(1);
              // }
            }
            tile.AddSample(Math::Vector2i{x, y}, Li, 1.0f);
          }
        }
        std::lock_guard<std::mutex> lk(mutex);
        film->MergeTile(tile);
      });
    });
  }

  std::shared_ptr<const Film> GetFilm() { return ctx.film; }

  std::future<void> future;
  RenderContext ctx;
  std::mutex mutex;
};

class Integrator {
 public:
  std::shared_ptr<RenderTask> CreateRenderTask(const RenderContext& ctx) {
    return std::make_shared<RenderTask>(ctx);
  }
};
}  // namespace Ajisai::Core
#endif
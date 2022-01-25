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
#include <ajisai/ajisai.h>
#include <ajisai/core/renderer/renderer.h>
#include <ajisai/core/sampler/sampler.h>
#include <ajisai/core/film.h>
#include <ajisai/core/parallel.h>
#include <ajisai/math/vector2.h>

AJ_BEGIN

void TiledIntegrator::Render(Scene* scene, Camera* camera, Film* film,
                             Sampler* sampler) {
  auto n_tiles =
      (film->Dimension() + Vector2i{tile_size_ - 1}) / Vector2i{tile_size_};
  auto beginTime = std::chrono::high_resolution_clock::now();
  parallel_for_2d(n_tiles, [=](Vector2i tile_pos, uint32_t tid) {
    (void)tid;
    Bounds2i tile_bounds = Bounds2i{tile_pos * (int)tile_size_,
                                    (tile_pos + Vector2i(1)) * (int)tile_size_};
    auto tile = film->GetTile(tile_bounds);
    auto tile_sampler = sampler->Copy();
    for (int y = tile.bounds.min().y(); y < tile.bounds.max().y(); ++y) {
      for (int x = tile.bounds.min().x(); x < tile.bounds.max().x(); ++x) {
        tile_sampler->SetSeed(x + y * film->Dimension().x());
        for (int s = 0; s < spp_; ++s) {
          const float u = (x + sampler->Next1D()) / film->Dimension().x();
          const float v = (y + sampler->Next1D()) / film->Dimension().y();
          //     // auto ray = camera->GenerateRay(u, v);
          //     auto Li = integrator->Li(scene.get(), camera.get(),
          //                              Math::Vector2i{x, y}, sampler.get());
          //     tile.AddSample(Math::Vector2i{x, y}, Li, 1.0f);
        }
      }
    }
  });
  auto endTime = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = (endTime - beginTime);
  std::cout << "Rendering done in " << elapsed.count() << "  secs\n";
}

class PathTracing : public TiledIntegrator {
 public:
  explicit PathTracing(const PTRendererArgs& args)
      : TiledIntegrator(args.spp, args.tile_size) {}

 private:
};

Rc<Renderer> CreatePTRenderer(const PTRendererArgs& args) {
  return RcNew<PathTracing>(args);
}

AJ_END
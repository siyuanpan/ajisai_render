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

#include <Ajisai/Core/RenderJob.h>

namespace Ajisai::Core {

RenderJob::RenderJob() {}

void RenderJob::Run() {
  future = std::async(std::launch::async, [=]() {
    auto beginTime = std::chrono::high_resolution_clock::now();

    auto& camera = ctx.camera;
    auto film = camera->GetFilm();
    auto scene = ctx.scene;
    auto& _sampler = ctx.sampler;
    auto integrator = ctx.integrator.get();
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
            auto Li = integrator->Li(ray, scene.get(), sampler.get());
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

  // return true;
}

}  // namespace Ajisai::Core
#ifndef AJISAI_CORE_FILM_H_
#define AJISAI_CORE_FILM_H_
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

#include <filesystem>
#include <vector>

#include "Ajisai/Core/Image.h"
#include "Ajisai/Core/Parallel.h"
#include "Ajisai/Math/Math.h"

namespace Ajisai::Core {

struct Pixel {
  Math::Spectrum radiance = Math::Spectrum(0);
  float weight = 0;
};

const size_t TileSize = 16;

struct Tile {
  Math::Bounds2i bounds{};
  Math::Vector2i size;
  std::vector<Pixel> pixels;
  explicit Tile(const Math::Bounds2i& bounds)
      : bounds{bounds}, size(bounds.Size()), pixels(size.x() * size.y()) {}

  auto& operator()(const Math::Vector2i& p) {
    auto q = p - bounds.min();
    return pixels[q.x() + q.y() * size.x()];
  }

  auto operator()(const Math::Vector2i& p) const {
    auto q = p - bounds.min();
    return pixels[q.x() + q.y() * size.x()];
  }

  void AddSample(const Math::Vector2i& p, const Math::Spectrum& radiance,
                 float weight) {
    auto& pix = (*this)(p);
    pix.radiance += radiance;
    pix.weight += weight;
  }
};

class Film {
 public:
  constexpr explicit Film(const Math::Vector2i& _dim)
      : dim{_dim}, radiance(_dim), weight(_dim) {}

  Math::Vector2i Dimension() const { return dim; }

  Tile GetTile(const Math::Bounds2i& bounds) const { return Tile{bounds}; }

  void MergeTile(const Tile& tile) {
    const auto lo = tile.bounds.min();
    const auto hi = tile.bounds.max();
    for (int y = lo.y(); y < hi.y(); ++y) {
      for (int x = lo.x(); x < hi.x(); ++x) {
        auto pix = tile(Math::Vector2i{x, y});
        radiance(x, y) += pix.radiance;
        weight(x, y) += pix.weight;
      }
    }
  }

  void WriteImage(const std::filesystem::path& path) const {
    RGBAImage image(Dimension());
    parallel_for(
        radiance.Dimension().y(),
        [&](uint32_t y, uint32_t) {
          for (int x = 0; x < radiance.Dimension().x(); ++x) {
            if (weight(x, y) != 0) {
              auto tmp = radiance(x, y) / weight(x, y);
              image(x, y) = Math::Color4f{tmp[0], tmp[1], tmp[2], 1.f};
            }
          }
        },
        1024);

    ImageWriter::Write(image, path);
  }

 private:
  Math::Vector2i dim;
  Image<Math::Spectrum> radiance;
  Image<float> weight;
};
}  // namespace Ajisai::Core

#endif
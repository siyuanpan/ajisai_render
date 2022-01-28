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
#pragma once
#include <ajisai/ajisai.h>
#include <ajisai/core/image.h>
#include <ajisai/core/parallel.h>
#include <ajisai/core/filter/filter.h>
#include <ajisai/math/spectrum.h>
#include <ajisai/math/bounds.h>
#include <ajisai/math/vector2.h>
#include <atomic>
#include <array>
#include <vector>
#include <filesystem>

AJ_BEGIN

class AtomicFloat {
  std::atomic<float> val;

 public:
  explicit AtomicFloat(float v = 0) : val(v) {}

  AtomicFloat(const AtomicFloat& rhs) : val((float)rhs.val) {}

  void Add(float v) {
    auto current = val.load();
    while (!val.compare_exchange_weak(current, current + v)) {
    }
  }

  [[nodiscard]] float Value() const { return val.load(); }

  explicit operator float() const { return Value(); }

  void Set(float v) { val = v; }
};

struct Pixel {
  Spectrum radiance = Spectrum{0.f};
  float weight = 0;
};

struct SplatPixel {
  std::array<AtomicFloat, 3> color;
  static_assert(sizeof(AtomicFloat) == sizeof(float));
  float padding{};
};
static_assert(sizeof(SplatPixel) == 4 * sizeof(float));

const size_t kTileSize = 16;

struct Tile {
  Bounds2i bounds{};
  Vector2i size;
  std::vector<Pixel> pixels;
  const Filter* filter;
  explicit Tile(const Bounds2i& bounds, const Filter* filter)
      : bounds{bounds},
        size(bounds.size()),
        pixels(size.x() * size.y()),
        filter(filter) {}

  auto& operator()(const Vector2i& p) {
    auto q = p - bounds.min();
    return pixels[q.x() + q.y() * size.x()];
  }

  auto operator()(const Vector2i& p) const {
    auto q = p - bounds.min();
    return pixels[q.x() + q.y() * size.x()];
  }

  void AddSample(const Vector2i& p, const Spectrum& radiance,
                 float weight = 1.f);
  // {
  //   auto& pix = (*this)(p);
  //   pix.radiance += radiance;
  //   pix.weight += weight;
  // }
};

class Film {
 public:
  constexpr explicit Film(const Vector2i& dim, const Filter* filter)
      : dim_{dim}, radiance(dim), weight(dim), splat(dim), filter_(filter) {}

  Vector2i Dimension() const { return dim_; }

  Tile GetTile(const Bounds2i& bounds) const;
  // {
  //   Bounds2i tile_pixel_bounds{};
  //   Vector2f half_pixel{0.5f};
  //   Bounds2f float_bounds{bounds};
  //   Ceil(float_bounds.min() - half_pixel - filter_->Radius());

  //   return Tile{bounds};
  // }

  void MergeTile(const Tile& tile) {
    const auto lo = tile.bounds.min();
    const auto hi = tile.bounds.max();
    for (int y = lo.y(); y < hi.y(); ++y) {
      for (int x = lo.x(); x < hi.x(); ++x) {
        auto pix = tile(Vector2i{x, y});
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
            Spectrum s = Spectrum{splat(x, y).color[0].Value(),
                                  splat(x, y).color[1].Value(),
                                  splat(x, y).color[2].Value()} *
                         SplatScale;
            if (weight(x, y) != 0) {
              auto tmp = (radiance(x, y)) / weight(x, y) + s;
              image(x, y) = Color4f{tmp[0], tmp[1], tmp[2], 1.f};
            } else {
              image(x, y) = Color4f{s[0], s[1], s[2], 1.f};
            }
          }
        },
        1024);

    ImageWriter::Write(image, path);
  }

  void AddSplat(const Spectrum& L, const Vector2f& p) {
    Vector2i ip{(int)p.x(), (int)p.y()};
    splat(ip.x(), ip.y()).color[0].Add(L[0]);
    splat(ip.x(), ip.y()).color[1].Add(L[1]);
    splat(ip.x(), ip.y()).color[2].Add(L[2]);
  }

  void ScaleToPixel(const float scale) { SplatScale = scale; }

 private:
  Vector2i dim_;
  Image<Spectrum> radiance;
  Image<float> weight;
  Image<SplatPixel> splat;
  float SplatScale = 1.f;
  const Filter* filter_;
};

AJ_END
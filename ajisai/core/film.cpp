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

#include <ajisai/core/film.h>

AJ_BEGIN

void Tile::AddSample(const Vector2i& p, const Spectrum& radiance,
                     float weight) {
  Vector2f half_pixel{0.5f};
  Vector2i p0 = Vector2i{Ceil(Vector2f{p} - half_pixel - filter->Radius())};
  Vector2i p1 = Vector2i{Floor(Vector2f{p} - half_pixel + filter->Radius())} +
                Vector2i{1};
  p0 = Max(p0, bounds.min());
  p1 = Min(p1, bounds.max());
  for (auto i = p0.y(); i < p1.y(); ++i) {
    for (auto j = p0.x(); j < p1.x(); ++j) {
      auto& pix = (*this)(Vector2i{j, i});
      auto filer_weight =
          filter->Eval(Vector2f{j - p.x() + 0.5f, i - p.y() + 0.5f});
      pix.weight += filer_weight;
      pix.radiance += weight * filer_weight * radiance;
    }
  }

  //   auto& pix = (*this)(p);
  //   pix.radiance += radiance;
  //   pix.weight += weight;
}

Tile Film::GetTile(const Bounds2i& bounds) const {
  Vector2f half_pixel{0.5f};
  Bounds2f float_bounds{bounds};
  Vector2i p0 =
      Vector2i{Ceil(float_bounds.min() - half_pixel - filter_->Radius())};
  Vector2i p1 =
      Vector2i{Floor(float_bounds.max() - half_pixel + filter_->Radius())} +
      Vector2i{1};
  Bounds2i tile_pixel_bounds =
      Intersect(Bounds2i{p0, p1}, Bounds2i{{0, 0}, dim_});

  return Tile{Bounds2i{p0, p1}, filter_};
  //   return Tile{bounds, filter_};
}

AJ_END
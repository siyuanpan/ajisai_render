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
#include <ajisai/core/filter/filter.h>
#include <ajisai/math/vector2.h>

AJ_BEGIN

class MitchellFilter : public Filter {
 public:
  explicit MitchellFilter(const Vector2f& radius, float B, float C)
      : radius_(radius), inv_radius_(1.f / radius), B(B), C(C) {}

  virtual Vector2f Radius() const noexcept override { return radius_; }

  virtual float Eval(const Vector2f& p) const noexcept override {
    return Mitchell1D(p.x() * inv_radius_.x()) *
           Mitchell1D(p.y() * inv_radius_.y());
  }

 private:
  float Mitchell1D(float x) const {
    x = std::abs(2 * x);
    if (x > 1)
      return ((-B - 6 * C) * x * x * x + (6 * B + 30 * C) * x * x +
              (-12 * B - 48 * C) * x + (8 * B + 24 * C)) *
             (1.f / 6.f);
    else
      return ((12 - 9 * B - 6 * C) * x * x * x +
              (-18 + 12 * B + 6 * C) * x * x + (6 - 2 * B)) *
             (1.f / 6.f);
  }

  Vector2f radius_, inv_radius_;
  float B, C;
};

Rc<Filter> CreateMitchellFilter(float xwidth, float ywidth, float B, float C) {
  return RcNew<MitchellFilter>(Vector2f{xwidth, ywidth}, B, C);
}

AJ_END
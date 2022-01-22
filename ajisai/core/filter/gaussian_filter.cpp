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

class GaussianFilter : public Filter {
 public:
  explicit GaussianFilter(const Vector2f& radius, float alpha)
      : radius_(radius),
        alpha_(alpha),
        expX_{std::exp(-alpha * radius.x() * radius.x())},
        expY_{std::exp(-alpha * radius.y() * radius.y())} {}

  virtual Vector2f Radius() const noexcept override { return radius_; }

  virtual float Eval(const Vector2f& p) const noexcept override {
    return Gaussian(p.x(), expX_) * Gaussian(p.y(), expY_);
  }

 private:
  float Gaussian(float d, float expv) const {
    return std::max(0.f, (float)std::exp(-alpha_ * d * d) - expv);
  }

  Vector2f radius_;
  float alpha_;
  float expX_, expY_;
};

Rc<Filter> CreateGaussianFilter(float xwidth, float ywidth, float alpha) {
  return RcNew<GaussianFilter>(Vector2f{xwidth, ywidth}, alpha);
}

AJ_END
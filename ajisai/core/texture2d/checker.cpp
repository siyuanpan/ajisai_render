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
#include <ajisai/core/texture2d/texture2d.h>

AJ_BEGIN

class Checker : public Texture2D {
 public:
  explicit Checker(const Spectrum& on_color, const Spectrum& off_color,
                   int res_u, int res_v)
      : on_color_(on_color),
        off_color_(off_color),
        res_u_(res_u),
        res_v_(res_v) {}

  virtual Spectrum SampleSpectrum(const Vector2f& uv) const noexcept override {
    Vector2i uv_i{(int)(uv.x() * (float)res_u_), (int)(uv.y() * (float)res_v_)};
    // std::cout << uv_i << std::endl;
    bool on = (uv_i.x() ^ uv_i.y()) & 1;
    return on ? on_color_ : off_color_;
  }

  virtual float SampleReal(const Vector2f& uv) const noexcept override {
    return SampleSpectrum(uv)[0];
  }

  virtual size_t Width() const noexcept override { return 1; }

  virtual size_t Height() const noexcept override { return 1; }

 private:
  Spectrum on_color_, off_color_;
  int res_u_, res_v_;
};

Rc<Texture2D> CreateCheckerTexture(const Spectrum& on_color,
                                   const Spectrum& off_color, int res_u,
                                   int res_v) {
  return RcNew<Checker>(on_color, off_color, res_u, res_v);
}

AJ_END
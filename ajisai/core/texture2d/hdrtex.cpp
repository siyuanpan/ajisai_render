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

class HDRTexture : public Texture2D {
 public:
  explicit HDRTexture(Rc<RGBImage>&& image_) : image_(image_) {}

  virtual Spectrum SampleSpectrum(const Vector2f& uv) const noexcept override {
    return LinearSampleImpl(uv);
    // return NearestSampleImpl(uv);
  }

  virtual float SampleReal(const Vector2f& uv) const noexcept override {
    return SampleSpectrum(uv)[0];
  }

  virtual size_t Width() const noexcept override {
    return image_->Dimension().x();
  }

  virtual size_t Height() const noexcept override {
    return image_->Dimension().y();
  }

 private:
  Spectrum NearestSampleImpl(const Vector2f& uv) const noexcept {
    Vector2f dim{(float)image_->Dimension().x(),
                 (float)image_->Dimension().y()};

    auto& image = *image_;
    auto coord = uv * dim;
    coord = clamp(coord, Vector2f{0.f}, dim - Vector2f{1.f});

    return Spectrum{image((int)coord[0], (int)coord[1])};
  }

  Spectrum LinearSampleImpl(const Vector2f& uv) const noexcept {
    Vector2f dim{(float)image_->Dimension().x(),
                 (float)image_->Dimension().y()};

    auto& image = *image_;

    auto coord = uv * dim - Vector2f{0.5f};

    coord = clamp(coord, Vector2f{0.f}, dim - Vector2f{1.f});

    Vector2f u0v0 = Floor(coord);
    Vector2f u1v1 = Ceil(coord);
    auto u0v1 = Vector2f{u0v0.x(), u1v1.y()};
    auto u1v0 = Vector2f{u1v1.x(), u0v0.y()};

    auto t = coord - u0v0;

    image((int)u0v0[0], (int)u0v0[1]);
    return LinearLerp(Spectrum{image((int)u0v0[0], (int)u0v0[1])},
                      Spectrum{image((int)u1v1[0], (int)u1v1[1])},
                      Spectrum{image((int)u0v1[0], (int)u0v1[1])},
                      Spectrum{image((int)u1v0[0], (int)u1v0[1])}, t);
  }

  Spectrum LinearLerp(const Spectrum& u0v0, const Spectrum& u1v1,
                      const Spectrum& u0v1, const Spectrum& u1v0,
                      const Vector2f& t) const {
    return (u0v0 * (1 - t[0]) + u1v0 * t[0]) * (1 - t[1]) +
           (u0v1 * (1 - t[0]) + u1v1 * t[0]) * t[1];
  }

 private:
  Rc<RGBImage> image_;
};

Rc<Texture2D> CreateHDRTexture(Rc<RGBImage>&& image) {
  return RcNew<HDRTexture>(std::move(image));
}

AJ_END
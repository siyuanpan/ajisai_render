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
#include <ajisai/core/parallel.h>
#include <ajisai/math/vector4.h>
#include <ajisai/math/color.h>
#include <filesystem>
#include <vector>

AJ_BEGIN

template <class T>
class Image {
 public:
  constexpr Image(const Vector2i& dim = Vector2i(1))
      : dimension_(dim), texels_(dim[0] * dim[1]) {}

  T& operator()(int x, int y) {
    x = std::clamp(x, 0, dimension_[0] - 1);
    y = std::clamp(y, 0, dimension_[1] - 1);
    return texels_[x + y * dimension_[0]];
  }

  const T operator()(int x, int y) const {
    x = std::clamp(x, 0, dimension_[0] - 1);
    y = std::clamp(y, 0, dimension_[1] - 1);
    return texels_[x + y * dimension_[0]];
  }

  void Resize(const Vector2i& size) {
    dimension_ = size;
    texels_.resize(dimension_[0] * dimension_[1]);
  }

  Vector2i Dimension() const { return dimension_; }

  const std::vector<T>& Texels() const { return texels_; }

 private:
  Vector2i dimension_;
  std::vector<T> texels_;
};

typedef Image<Vector3f> SRGBImage;
typedef Image<Vector4f> SRGBAImage;
typedef Image<Color3f> RGBImage;
typedef Image<Color4f> RGBAImage;

class AJISAI_API ImageWriter {
 public:
  static bool Write(const RGBAImage& image, const std::filesystem::path& path);

  static void GammaCorrection(const RGBAImage& in, SRGBAImage& out,
                              float gamma = 1.0 / 2.2f) {
    out.Resize(in.Dimension());
    parallel_for(
        in.Dimension().y(),
        [&](uint32_t y, uint32_t) {
          for (int x = 0; x < in.Dimension().x(); ++x) {
            out(x, y) = in(x, y).toSrgba();
          }
        },
        1024);
  }
};

AJ_END
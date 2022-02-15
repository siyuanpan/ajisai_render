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

#ifndef AJISAI_CORE_IMAGE_H_
#define AJISAI_CORE_IMAGE_H_

#include <filesystem>
#include <iostream>

// #define STB_IMAGE_WRITE_IMPLEMENTATION
// #include <stb_image.h>
// #include <stb_image_write.h>

#include "Ajisai/Core/Parallel.h"
#include "Ajisai/Math/Color.h"
#include "Ajisai/Math/Math.h"

namespace Ajisai::Core {
template <class T>
class Image {
 public:
  constexpr Image(const Math::Vector2i& dim = Math::Vector2i(1))
      : dimension(dim), texels(dim[0] * dim[1]) {}

  auto& operator()(int x, int y) {
    x = std::clamp(x, 0, dimension[0] - 1);
    y = std::clamp(y, 0, dimension[1] - 1);
    return texels[x + y * dimension[0]];
  }

  auto operator()(int x, int y) const {
    x = std::clamp(x, 0, dimension[0] - 1);
    y = std::clamp(y, 0, dimension[1] - 1);
    return texels[x + y * dimension[0]];
  }

  void Resize(const Math::Vector2i& size) {
    dimension = size;
    texels.resize(dimension[0] * dimension[1]);
  }

  Math::Vector2i Dimension() const { return dimension; }

  const std::vector<T>& Texels() const { return texels; }

 private:
  Math::Vector2i dimension;
  std::vector<T> texels;
};

typedef Image<Math::Vector3f> SRGBImage;
typedef Image<Math::Vector4f> SRGBAImage;
typedef Image<Math::Color3f> RGBImage;
typedef Image<Math::Color4f> RGBAImage;

class ImageWriter {
 public:
  static bool Write(const RGBAImage& _image, const std::filesystem::path& path);
  // static bool Write(const RGBAImage& _image,
  //                   const std::filesystem::path& path) {
  //   const auto ext = path.extension().string();
  //   std::cout << ext.c_str() << std::endl;
  //   SRGBAImage image;
  //   GammaCorrection(_image, image);
  //   auto& texels = image.Texels();
  //   auto dimension = image.Dimension();
  //   std::vector<uint8_t> buffer(texels.size() * 3);
  //   parallel_for(
  //       texels.size(),
  //       [&](uint32_t i, uint32_t) {
  //         auto pix = static_cast<uint8_t*>(&buffer[i * 3]);
  //         auto rgb = texels[i].rgb();
  //         rgb = Math::clamp(rgb, 0.f, 1.f);
  //         for (int j = 0; j < 3; ++j) {
  //           pix[j] =
  //               (uint8_t)std::clamp<int>(std::round(rgb[j] * 255.5), 0, 255);
  //           // pix[j] = (uint8_t)std::clamp(255 * rgb[j] + 0.5f, 0.f, 255.f);
  //         }
  //       },
  //       1024);

  //   stbi_flip_vertically_on_write(true);
  //   if (ext == ".png")
  //     return stbi_write_png(path.string().c_str(), dimension.x(),
  //     dimension.y(),
  //                           3, buffer.data(), 0);
  //   else if (ext == ".jpg")
  //     return stbi_write_jpg(path.string().c_str(), dimension.x(),
  //     dimension.y(),
  //                           3, buffer.data(), 0);
  //   return false;
  // }

  static void GammaCorrection(const RGBAImage& in, SRGBAImage& out,
                              float gamma = 1.0 / 2.2f) {
    out.Resize(in.Dimension());
    parallel_for(
        in.Dimension().y(),
        [&](uint32_t y, uint32_t) {
          for (int x = 0; x < in.Dimension().x(); ++x) {
            out(x, y) = in(x, y).toSrgba();
            // Math::Vector4f(pow(in(x, y).xyz(), gamma), in(x, y).w());
          }
        },
        1024);
  }
};
}  // namespace Ajisai::Core

#endif
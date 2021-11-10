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

#include <Ajisai/Core/Image.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image.h>
#include <stb_image_write.h>

namespace Ajisai::Core {
bool ImageWriter::Write(const RGBAImage& _image,
                        const std::filesystem::path& path) {
  const auto ext = path.extension().string();
  std::cout << ext.c_str() << std::endl;
  SRGBAImage image;
  GammaCorrection(_image, image);
  auto& texels = image.Texels();
  auto dimension = image.Dimension();
  std::vector<uint8_t> buffer(texels.size() * 3);
  parallel_for(
      texels.size(),
      [&](uint32_t i, uint32_t) {
        auto pix = static_cast<uint8_t*>(&buffer[i * 3]);
        auto rgb = texels[i].rgb();
        rgb = Math::clamp(rgb, 0.f, 1.f);
        for (int j = 0; j < 3; ++j) {
          pix[j] = (uint8_t)std::clamp<int>(std::round(rgb[j] * 255.5), 0, 255);
          // pix[j] = (uint8_t)std::clamp(255 * rgb[j] + 0.5f, 0.f, 255.f);
        }
      },
      1024);

  stbi_flip_vertically_on_write(true);
  if (ext == ".png")
    return stbi_write_png(path.string().c_str(), dimension.x(), dimension.y(),
                          3, buffer.data(), 0);
  else if (ext == ".jpg")
    return stbi_write_jpg(path.string().c_str(), dimension.x(), dimension.y(),
                          3, buffer.data(), 0);
  return false;
}
}  // namespace Ajisai::Core
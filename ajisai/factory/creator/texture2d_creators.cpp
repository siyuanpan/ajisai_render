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
#include <ajisai/factory/creator/texture2d_creators.h>
#include <ajisai/factory/creator/helper.h>
#include <ajisai/math/color.h>

AJ_BEGIN

namespace {
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
}  // namespace

class ConstantCreatorImpl {
 public:
  static std::string Name() { return "constant"; }

  static Rc<Texture2D> Create(const YAML::Node& node,
                              const CreateFactory& factory) {
    const auto texel = node["texel"].as<Vector3f>();

    return CreateConstant2DTexture(Spectrum{texel});
  }
};

class HdrCreatorImpl {
 public:
  static std::string Name() { return "hdr"; }

  static Rc<Texture2D> Create(const YAML::Node& node,
                              const CreateFactory& factory) {
    const auto filename = node["filename"].as<std::string>();

    int w, h, channels;
    float* data = stbi_loadf(filename.c_str(), &w, &h, &channels, STBI_rgb);

    if (!data) {
      AJ_ERROR("load file {} error", filename);
      std::exit(1);
    }

    auto image = RcNew<RGBImage>(Vector2i{w, h});

    parallel_for(
        image->Dimension().y(),
        [&](uint32_t y, uint32_t) {
          for (int x = 0; x < image->Dimension().x(); ++x) {
            int img_idx = y * w * 3 + x * 3;
            (*image)(x, y) = Spectrum{data[img_idx + 0], data[img_idx + 1],
                                      data[img_idx + 2]};
          }
        },
        1024);

    stbi_image_free(data);

    return CreateHDRTexture(std::move(image));
  }
};

template <class TTexture2DCreatorImpl>
concept Texture2DCreatorImpl = requires(TTexture2DCreatorImpl) {
  { TTexture2DCreatorImpl::Name() } -> std::convertible_to<std::string>;
  {
    TTexture2DCreatorImpl::Create(YAML::Node{}, CreateFactory{})
    } -> std::convertible_to<Rc<Texture2D>>;
};

template <Texture2DCreatorImpl TTexture2DCreatorImpl>
class Texture2DCreator : public TTexture2DCreatorImpl {};

using ConstantCreator = Texture2DCreator<ConstantCreatorImpl>;
using HdrCreator = Texture2DCreator<HdrCreatorImpl>;

void AddTexture2DFactory(Factory<Texture2D>& factory) {
  factory.Add(ConstantCreator::Name(), &ConstantCreator::Create);
  factory.Add(HdrCreator::Name(), &HdrCreator::Create);
}

AJ_END
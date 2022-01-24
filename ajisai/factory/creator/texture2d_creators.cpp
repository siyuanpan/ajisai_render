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

class ConstantCreatorImpl {
 public:
  static std::string Name() { return "constant"; }

  static Rc<Texture2D> Create(const YAML::Node& node,
                              const CreateFactory& factory) {
    const auto texel = node["texel"].as<Vector3f>();

    return CreateConstant2DTexture(Spectrum{texel});
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

void AddTexture2DFactory(Factory<Texture2D>& factory) {
  factory.Add(ConstantCreator::Name(), &ConstantCreator::Create);
}

AJ_END
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
#include <ajisai/factory/creator/primitive_creators.h>
#include <ajisai/factory/creator/helper.h>

AJ_BEGIN

MediumInterface CreateMediumInterface(const YAML::Node& node,
                                      const CreateFactory& factory) {
  MediumInterface ret{};

  if (auto in = node["med_in"]) {
    ret.in = factory.Create<Medium>(in);
  } else {
    ret.in = CreateVoidMedium();
  }

  if (auto out = node["med_out"]) {
    ret.out = factory.Create<Medium>(out);
  } else {
    ret.out = CreateVoidMedium();
  }

  return ret;
}

class GeometricPrimitiveCreatorImpl {
 public:
  static std::string Name() { return "geometry"; }

  static Rc<Primitive> Create(const YAML::Node& node,
                              const CreateFactory& factory) {
    auto geometry = factory.Create<Geometry>(node["geometry"]);

    Rc<Material> material;
    if (auto mat = node["material"]) {
      material = factory.Create<Material>(mat);
    } else {
      material = CreateNull();
    }

    // auto material = factory.Create<Material>(node["material"]);

    const auto medium = CreateMediumInterface(node, factory);

    const auto emission = node["emission"].as<Vector3f>(Vector3f{});

    const auto denoise = node["denoise"].as<bool>(false);

    const int32_t power = node["power"].as<int32_t>(-1);

    return CreateGeometric(std::move(geometry), std::move(material), medium,
                           emission, denoise, power);
  }
};

template <class TPrimitiveCreatorImpl>
concept PrimitiveCreatorImpl = requires(TPrimitiveCreatorImpl) {
  { TPrimitiveCreatorImpl::Name() } -> std::convertible_to<std::string>;
  {
    TPrimitiveCreatorImpl::Create(YAML::Node{}, CreateFactory{})
    } -> std::convertible_to<Rc<Primitive>>;
};

template <PrimitiveCreatorImpl TPrimitiveCreatorImpl>
class PrimitiveCreator : public TPrimitiveCreatorImpl {};

using GeometricPrimitiveCreator =
    PrimitiveCreator<GeometricPrimitiveCreatorImpl>;

void AddPrimitiveFactory(Factory<Primitive>& factory) {
  factory.Add(GeometricPrimitiveCreator::Name(),
              &GeometricPrimitiveCreator::Create);
}

AJ_END
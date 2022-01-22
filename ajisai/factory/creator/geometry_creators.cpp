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
#include <ajisai/factory/creator/geometry_creators.h>
#include <ajisai/factory/creator/helper.h>

AJ_BEGIN

class QuadCreatorImpl {
 public:
  static std::string Name() { return "quad"; }

  static Rc<Geometry> Create(const YAML::Node& node,
                             const CreateFactory& factory) {
    const auto transform = GetTransform(node["transform"]);

    const Vector3f a = node["a"].as<Vector3f>(Vector3f{-1.f, -1.f, 0.f});
    const Vector3f b = node["b"].as<Vector3f>(Vector3f{1.f, -1.f, 0.f});
    const Vector3f c = node["c"].as<Vector3f>(Vector3f{1.f, 1.f, 0.f});
    const Vector3f d = node["d"].as<Vector3f>(Vector3f{-1.f, 1.f, 0.f});

    const Vector2f ta = node["ta"].as<Vector2f>(Vector2f{});
    const Vector2f tb = node["tb"].as<Vector2f>(Vector2f{});
    const Vector2f tc = node["tc"].as<Vector2f>(Vector2f{});
    const Vector2f td = node["td"].as<Vector2f>(Vector2f{});

    // std::stringstream ss;
    // ss << "---\n"
    //    << "a : " << a << "\nb : " << b << "\nc : " << c << "\nd : " << d
    //    << "\nta : " << ta << "\ntb : " << tb << "\ntc : " << tc
    //    << "\ntd : " << td << "\ntransform : " << transform;

    // AJ_DEBUG("create quad with param : {}", ss.str());
    return CreateQuad(a, b, c, d, ta, tb, tc, td, transform);
  }
};

class CubeCreatorImpl {
 public:
  static std::string Name() { return "cube"; }

  static Rc<Geometry> Create(const YAML::Node& node,
                             const CreateFactory& factory) {
    const auto transform = GetTransform(node["transform"]);

    return CreateCube(transform);
  }
};

class TwosidedCreatorImpl {
 public:
  static std::string Name() { return "twosided"; }

  static Rc<Geometry> Create(const YAML::Node& node,
                             const CreateFactory& factory) {
    const auto internal = factory.Create<Geometry>(node["internal"]);

    return CreateTwoSided(std::move(internal));
  }
};

template <class TGeometryCreatorImpl>
concept GeometryCreatorImpl = requires(TGeometryCreatorImpl) {
  { TGeometryCreatorImpl::Name() } -> std::convertible_to<std::string>;
  {
    TGeometryCreatorImpl::Create(YAML::Node{}, CreateFactory{})
    } -> std::convertible_to<Rc<Geometry>>;
};

template <GeometryCreatorImpl TGeometryCreatorImpl>
class GeometryCreator : public TGeometryCreatorImpl {};

using QuadCreator = GeometryCreator<QuadCreatorImpl>;
using CubeCreator = GeometryCreator<CubeCreatorImpl>;
using TwosidedCreator = GeometryCreator<TwosidedCreatorImpl>;

void AddGeometricFactory(Factory<Geometry>& factory) {
  factory.Add(QuadCreator::Name(), &QuadCreator::Create);
  factory.Add(CubeCreator::Name(), &CubeCreator::Create);
  factory.Add(TwosidedCreator::Name(), &TwosidedCreator::Create);
}

AJ_END
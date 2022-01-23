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

AJ_BEGIN

class QuadCreatorImpl {
 public:
  static std::string Name() { return "quad"; }

  static Rc<Geometric> Create(const YAML::Node& node,
                              const CreateFactory& factory) {
    // auto geometric = factory.Create<Geometric>(node["geometric"]);

    return RcNew<Geometric>();
  }
};

class CubeCreatorImpl {
 public:
  static std::string Name() { return "cube"; }

  static Rc<Geometric> Create(const YAML::Node& node,
                              const CreateFactory& factory) {
    // auto geometric = factory.Create<Geometric>(node["geometric"]);

    return RcNew<Geometric>();
  }
};

template <class TGeometryCreatorImpl>
concept GeometryCreatorImpl = requires(TGeometryCreatorImpl) {
  { TGeometryCreatorImpl::Name() } -> std::convertible_to<std::string>;
  {
    TGeometryCreatorImpl::Create(YAML::Node{}, CreateFactory{})
    } -> std::convertible_to<Rc<Geometric>>;
};

template <GeometryCreatorImpl TGeometryCreatorImpl>
class GeometryCreator : public TGeometryCreatorImpl {};

using QuadCreator = GeometryCreator<QuadCreatorImpl>;
using CubeCreator = GeometryCreator<CubeCreatorImpl>;

void AddGeometricFactory(Factory<Geometric>& factory) {
  factory.Add(QuadCreator::Name(), &QuadCreator::Create);
  factory.Add(CubeCreator::Name(), &CubeCreator::Create);
}

AJ_END
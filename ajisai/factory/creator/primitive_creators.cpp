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

AJ_BEGIN

class GeometricPrimitiveCreatorImpl {
 public:
  static std::string Name() { return "geometric"; }

  static Rc<Primitive> Create(const YAML::Node& node,
                              const CreateFactory& factory) {
    auto geometric = factory.Create<Geometric>(node["geometric"]);

    return RcNew<Primitive>();
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
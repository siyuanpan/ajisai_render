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

#ifndef AJISAI_MATERIALS_MATERIAL_H_
#define AJISAI_MATERIALS_MATERIAL_H_

// #include "Ajisai/Math/Math.h"

namespace Ajisai::Core {

struct SurfaceInteraction;

}

namespace Ajisai::Materials {
class Material {
 public:
  // enum Type { Diffuse };
  Material() {}
  // Material(const Math::Color3<float> c) : color(c), type(t) {}
  // Math::Color3<float> color;
  // Type type;

  virtual void ComputeScatteringFunction(
      Core::SurfaceInteraction* si) const = 0;
};
}  // namespace Ajisai::Materials

#endif
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
#include <ajisai/core/coordinate_system.h>
#include <ajisai/math/vector3.h>
#include <ajisai/math/spectrum.h>

#include <vector>

AJ_BEGIN
class BSDFComponent;

class BSDF {
 public:
  BSDF(const Vector3f& ng, const Vector3f& ns, const Spectrum& albedo)
      : geometry_normal_(ng), frame_{ns}, albedo_{albedo} {}

  void AddComponent(float weight, Rc<BSDFComponent> component);

  Spectrum albedo() const;

 private:
  Vector3f geometry_normal_;
  const CoordinateSystem frame_;
  Spectrum albedo_;
  std::vector<Rc<BSDFComponent>> components_;
  std::vector<float> weights_;
};

class BSDFComponent {
 public:
  virtual ~BSDFComponent() = default;
};

AJ_END
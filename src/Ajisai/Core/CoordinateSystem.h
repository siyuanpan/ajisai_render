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

#ifndef AJISAI_CORE_COORDINATESYSTEM_H_
#define AJISAI_CORE_COORDINATESYSTEM_H_

#include "Ajisai/Math/Math.h"

namespace Ajisai::Core {
class CoordinateSystem {
 public:
  CoordinateSystem() = default;
  CoordinateSystem(const Math::Vector3f& v) : n(v) {
    if (std::abs(n.x()) > std::abs(n.y())) {
      float invLen = 1.0f / std::sqrt(n.x() * n.x() + n.z() * n.z());
      s = Math::Vector3f{n.z(), 0.f, -n.x()} * invLen;
    } else {
      float invLen = 1.0f / std::sqrt(n.y() * n.y() + n.z() * n.z());
      s = Math::Vector3f{0.0f, n.z(), -n.y()} * invLen;
    }
    t = Math::cross(s, n);
  }

  Math::Vector3f toLocal(const Math::Vector3f& v) const {
    return {Math::dot(v, s), Math::dot(v, t), Math::dot(v, n)};
  }

  Math::Vector3f toWorld(const Math::Vector3f& v) const {
    return s * v.x() + t * v.y() + n * v.z();
  }

 private:
  Math::Vector3f n;
  Math::Vector3f s;
  Math::Vector3f t;
};

}  // namespace Ajisai::Core

#endif
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
#include <ajisai/math/vector3.h>
#include <cmath>

AJ_BEGIN

class CoordinateSystem {
 public:
  CoordinateSystem() = default;

  /* Based on "Building an Orthonormal Basis, Revisited" by
     Tom Duff, James Burgess, Per Christensen,
     Christophe Hery, Andrew Kensler, Max Liani,
     and Ryusuke Villemin (JCGT Vol 6, No 1, 2017) */
  CoordinateSystem(const Vector3f& n) : dz(n) {
    float sign = copysignf(1.f, n.z());
    const float a = -1.f / (sign + n.z());
    const float b = n.x() * n.y() * a;

    dx = Vector3f{1.f + sign * n.x() * n.x() * a, sign * b, -sign * n.x()};
    dy = Vector3f{b, sign + n.y() * n.y() * a, -n.y()};
  }

  Vector3f World2Local(const Vector3f& v) const {
    return {dot(v, dx), dot(v, dy), dot(v, dz)};
  }

  Vector3f Local2World(const Vector3f& v) const {
    return dx * v.x() + dy * v.y() + dz * v.z();
  }

  Vector3f x() const { return dx; }

  Vector3f y() const { return dy; }

  Vector3f z() const { return dz; }

 private:
  Vector3f dx;
  Vector3f dy;
  Vector3f dz;
};

AJ_END
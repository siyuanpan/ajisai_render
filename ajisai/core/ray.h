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

AJ_BEGIN

struct Ray {
  Vector3f o, d;
  float t_min, t_max;

  Ray() = default;
  Ray(const Vector3f& o, const Vector3f& d,
      float t_min = Ray::Eps(),  // 0.001 /*ray bias*/,
      float t_max = std::numeric_limits<float>::infinity())
      : o(o), d(d.normalized()), t_min(t_min), t_max(t_max) {}

  auto CalcPoint(float t) const { return o + t * d; }

  static inline float Eps() { return 1e-4f; }
  static inline float ShadowEps() { return 1e-4f; }
};

AJ_END
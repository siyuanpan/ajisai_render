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
#include <ajisai/math/math.h>

AJ_BEGIN

template <class>
struct Constants;

template <>
struct Constants<float> {
  Constants() = delete;

  // https://en.wikipedia.org/wiki/Single-precision_floating-point_format
  static constexpr float pi() { return 3.141592654f; }
  static constexpr float pi2() { return 1.570796327f; }
  static constexpr float pi4() { return 0.785398163f; }
  static constexpr float tau() { return 6.283185307f; }
  static constexpr float e() { return 2.718281828f; }
  static constexpr float sqrt2() { return 1.414213562f; }
  static constexpr float sqrt3() { return 1.732050808f; }
  static constexpr float sqrtHalf() { return 0.707106781f; }
};

template <>
struct Constants<double> {
  Constants() = delete;

  // https://en.wikipedia.org/wiki/Double-precision_floating-point_format
  static constexpr double pi() { return 3.1415926535897932; }
  static constexpr double pi2() { return 1.5707963267948966; }
  static constexpr double pi4() { return 0.7853981633974483; }
  static constexpr double tau() { return 6.2831853071795864; }
  static constexpr double e() { return 2.7182818284590452; }
  static constexpr double sqrt2() { return 1.4142135623730950; }
  static constexpr double sqrt3() { return 1.7320508075688773; }
  static constexpr double sqrtHalf() { return 0.7071067811865475; }
};

AJ_END
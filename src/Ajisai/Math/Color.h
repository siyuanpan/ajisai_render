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

#ifndef AJISAI_MATH_COLOR_H_
#define AJISAI_MATH_COLOR_H_

#include "Ajisai/Math/Vector3.h"

namespace Ajisai::Math {

template <class>
class Color3;

namespace Impl {

/* sRGB -> RGB */
template <class T>
Color3<T> fromSrgb(const Vector3<T>& srgb) {
  constexpr const T a(T(0.055));
  return lerp(srgb / T(12.92),
              pow((srgb + Vector3<T>(a)) / (T(1.0) + a), T(2.4)),
              srgb > Vector3<T>(T(0.04045)));
}

/* RGB -> sRGB */
template <class T>
Vector3<T> toSrgb(const Color3<T>& rgb) {
  constexpr const T a = T(0.055);
  return lerp(rgb * T(12.92),
              (T(1.0) + a) * pow(rgb, T(1.0) / T(2.4)) - Vector3<T>{a},
              rgb > Vector3<T>(T(0.0031308)));
}
}  // namespace Impl

template <class T>
class Color3 : public Vector3<T> {
 public:
  //   typedef typename TypeTraits<T>::FloatingPointType FloatingPointType;
  static Color3<T> fromSrgb(const Vector3<T>& srgb) {
    return Impl::fromSrgb<T>(srgb);
  }

  constexpr Color3() noexcept : Vector3<T>{} {}

  constexpr explicit Color3(T rgb) noexcept : Vector3<T>(rgb) {}

  constexpr Color3(T r, T g, T b) noexcept : Vector3<T>(r, g, b) {}

  constexpr Color3(const Vector<T, 3>& other) noexcept : Vector3<T>(other) {}

  Vector3<T> toSrgb() const { return Impl::toSrgb(*this); }

  VECTOR_SUBCLASS_OPERATOR_IMPL(Color3, 3)
};

VECTOR_FUNCTION_IMPL(Color3, 3)

}  // namespace Ajisai::Math

#endif
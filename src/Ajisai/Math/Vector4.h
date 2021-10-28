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

#ifndef AJISAI_MATH_VECTOR4_H_
#define AJISAI_MATH_VECTOR4_H_

#include "Ajisai/Math/Vector3.h"

namespace Ajisai::Math {

template <class T>
class Vector4 : public Vector<T, 4> {
 public:
  constexpr Vector4() noexcept : Vector<T, 4>{} {}

  constexpr explicit Vector4(T value) noexcept : Vector<T, 4>(value) {}

  constexpr Vector4(T x, T y, T z, T w) noexcept : Vector<T, 4>(x, y, z, w) {}

  constexpr Vector4(const Vector3<T>& xyz, T w) noexcept
      : Vector<T, 4>(xyz[0], xyz[1], xyz[2], w) {}

  constexpr Vector4(const Vector<T, 4>& other) noexcept : Vector<T, 4>(other) {}

  T& x() { return Vector<T, 4>::_data[0]; }
  constexpr T x() const { return Vector<T, 4>::_data[0]; }
  T& y() { return Vector<T, 4>::_data[1]; }
  constexpr T y() const { return Vector<T, 4>::_data[1]; }
  T& z() { return Vector<T, 4>::_data[2]; }
  constexpr T z() const { return Vector<T, 4>::_data[2]; }
  T& w() { return Vector<T, 4>::_data[3]; }
  constexpr T w() const { return Vector<T, 4>::_data[3]; }

  Vector3<T>& xyz() { return Vector3<T>::from(Vector<T, 4>::data()); }
  constexpr const Vector3<T> xyz() const {
    return {Vector<T, 4>::_data[0], Vector<T, 4>::_data[1],
            Vector<T, 4>::_data[2]};
  }

  Vector3<T>& rgb() { return Vector3<T>::from(Vector<T, 4>::data()); }
  constexpr const Vector3<T> rgb() const {
    return {Vector<T, 4>::_data[0], Vector<T, 4>::_data[1],
            Vector<T, 4>::_data[2]};
  }

  VECTOR_SUBCLASS_OPERATOR_IMPL(Vector4, 4)
};

}  // namespace Ajisai::Math

#endif
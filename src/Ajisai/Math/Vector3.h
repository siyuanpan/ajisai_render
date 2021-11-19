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

#ifndef AJISAI_MATH_VECTOR3_H_
#define AJISAI_MATH_VECTOR3_H_

#include "Ajisai/Math/Vector2.h"

namespace Ajisai::Math {

// template <class>
// class Vector3;

// template <class U>
// inline Vector3<U> cross(const Vector3<U>& a, const Vector3<U>& b) {
//   return {a._data[1] * b._data[2] - b._data[1] * a._data[2],
//           a._data[2] * b._data[0] - b._data[2] * a._data[0],
//           a._data[0] * b._data[1] - b._data[0] * a._data[1]};
// }

template <class T>
class Vector3 : public Vector<T, 3> {
 public:
  constexpr Vector3() noexcept : Vector<T, 3>{} {}

  constexpr explicit Vector3(T value) noexcept : Vector<T, 3>(value) {}

  constexpr Vector3(T x, T y, T z) noexcept : Vector<T, 3>(x, y, z) {}

  constexpr Vector3(const Vector<T, 3>& other) noexcept : Vector<T, 3>(other) {}

  T& x() { return Vector<T, 3>::_data[0]; }
  constexpr T x() const { return Vector<T, 3>::_data[0]; }
  T& y() { return Vector<T, 3>::_data[1]; }
  constexpr T y() const { return Vector<T, 3>::_data[1]; }
  T& z() { return Vector<T, 3>::_data[2]; }
  constexpr T z() const { return Vector<T, 3>::_data[2]; }

  T& r() { return Vector<T, 3>::_data[0]; }
  constexpr T r() const { return Vector<T, 3>::_data[0]; }
  T& g() { return Vector<T, 3>::_data[1]; }
  constexpr T g() const { return Vector<T, 3>::_data[1]; }
  T& b() { return Vector<T, 3>::_data[2]; }
  constexpr T b() const { return Vector<T, 3>::_data[2]; }

  VECTOR_SUBCLASS_OPERATOR_IMPL(Vector3, 3)

 private:
  template <class U>
  friend Vector3<U> cross(const Vector3<U>&, const Vector3<U>&);
};

VECTOR_FUNCTION_IMPL(Vector3, 3)

template <class U>
inline Vector3<U> cross(const Vector3<U>& a, const Vector3<U>& b) {
  return {a._data[1] * b._data[2] - b._data[1] * a._data[2],
          a._data[2] * b._data[0] - b._data[2] * a._data[0],
          a._data[0] * b._data[1] - b._data[0] * a._data[1]};
}

}  // namespace Ajisai::Math

#endif
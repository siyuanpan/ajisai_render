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

#ifndef AJISAI_MATH_MATRIX4_H_
#define AJISAI_MATH_MATRIX4_H_

#include <Ajisai/Math/Matrix.h>
#include <Ajisai/Math/Vector4.h>

namespace Ajisai::Math {

template <class T>
class Matrix4 : public Matrix4x4<T> {
 public:
  constexpr static Matrix4<T> translation(const Vector3<T>& vector) {
    return {{T(1), T(0), T(0), T(0)},
            {T(0), T(1), T(0), T(0)},
            {T(0), T(0), T(1), T(0)},
            {vector.x(), vector.y(), vector.z(), T(1)}};
  }

  static Matrix4<T> lookAt(const Vector3<T>& eye, const Vector3<T>& target,
                           const Vector3<T>& up);

  constexpr static Matrix4<T> from(const Matrix3x3<T>& rotationScaling,
                                   const Vector3<T>& translation) {
    return {{rotationScaling[0], T(0)},
            {rotationScaling[1], T(0)},
            {rotationScaling[2], T(0)},
            {translation, T(1)}};
  }

  constexpr Matrix4() noexcept : Matrix4x4<T>{IdentityInit, T(1)} {}

  constexpr explicit Matrix4(IdentityInitT, T value = T{1}) noexcept
      : Matrix4x4<T>{IdentityInit, value} {}

  constexpr Matrix4(const Vector4<T>& first, const Vector4<T>& second,
                    const Vector4<T>& third, const Vector4<T>& fourth) noexcept
      : Matrix4x4<T>{first, second, third, fourth} {}

  constexpr explicit Matrix4(T value) noexcept : Matrix4x4<T>{value} {}

  template <class U>
  constexpr explicit Matrix4(const RectangularMatrix<U, 4, 4>& other) noexcept
      : Matrix4x4<T>(other) {}

  //   template <std::size_t otherCols, std::size_t otherRows>
  //   constexpr explicit Matrix4(
  //       IdentityInitT, const RectangularMatrix<T, otherCols, otherRows>&
  //       other, T value = T(1)) noexcept : Matrix4x4<T>{IdentityInit, other,
  //       value} {}

  template <std::size_t otherCols, std::size_t otherRows>
  constexpr explicit Matrix4(
      const RectangularMatrix<T, otherCols, otherRows>& other) noexcept
      : Matrix4x4<T>{other} {}

  constexpr Matrix4(const RectangularMatrix<T, 4, 4>& other) noexcept
      : Matrix4x4<T>{other} {}
};

template <class T>
Matrix4<T> Matrix4<T>::lookAt(const Vector3<T>& eye, const Vector3<T>& target,
                              const Vector3<T>& up) {
  const Vector3<T> backward = (eye - target).normalized();
  const Vector3<T> right = cross(up, backward).normalized();
  const Vector3<T> realUp = cross(backward, right).normalized();
  return from({right, realUp, backward}, eye);
}

}  // namespace Ajisai::Math

#endif
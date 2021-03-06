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

#include <ajisai/math/vector.h>

AJ_BEGIN

template <class T>
class Vector2 : public Vector<T, 2> {
 public:
  constexpr static Vector2<T> xScale(T scale) { return {scale, T(1)}; }

  constexpr static Vector2<T> yScale(T scale) { return {T(1), scale}; }

  constexpr Vector2() noexcept : Vector<T, 2>{} {}

  constexpr explicit Vector2(T value) noexcept : Vector<T, 2>(value) {}

  constexpr Vector2(T x, T y) noexcept : Vector<T, 2>(x, y) {}

  template <class U>
  constexpr explicit Vector2(const Vector<U, 2>& other) noexcept
      : Vector<T, 2>(other) {}

  constexpr Vector2(const Vector<T, 2>& other) noexcept : Vector<T, 2>(other) {}

  T& x() { return Vector<T, 2>::_data[0]; }
  constexpr T x() const { return Vector<T, 2>::_data[0]; }
  T& y() { return Vector<T, 2>::_data[1]; }
  constexpr T y() const { return Vector<T, 2>::_data[1]; }

  template <class U = T>
  typename std::enable_if<std::is_floating_point<U>::value, T>::type
  AspectRatio() const {
    return x() / y();
  }

  VECTOR_SUBCLASS_OPERATOR_IMPL(Vector2, 2)
};

VECTOR_FUNCTION_IMPL(Vector2, 2)

AJ_END
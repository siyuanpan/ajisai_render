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

#ifndef AJISAI_MATH_MATRIX_H_
#define AJISAI_MATH_MATRIX_H_

#include <Ajisai/Math/RectangularMatrix.h>

namespace Ajisai::Math {

namespace {

template <std::size_t size, std::size_t col, std::size_t otherSize, class T,
          std::size_t... row>
Vector<T, size> valueOrIdentityVector(
    std::index_sequence<row...>,
    const RectangularMatrix<T, otherSize, otherSize>& other) {
  return {(col < otherSize && row < otherSize ? other[col][row]
           : col == row                       ? T{1}
                                              : T{0})...};
}

template <std::size_t size, std::size_t col, std::size_t otherSize, class T>
Vector<T, size> valueOrIdentityVector(
    const RectangularMatrix<T, otherSize, otherSize>& other) {
  return valueOrIdentityVector<size, col>(std::make_index_sequence<size>(),
                                          other);
}

}  // namespace

template <class T, std::size_t size>
class Matrix : public RectangularMatrix<T, size, size> {
 public:
  constexpr Matrix() noexcept : RectangularMatrix<T, size, size>{} {}

  constexpr explicit Matrix(IdentityInitT, T value = T(1)) noexcept
      : RectangularMatrix<T, size, size>{std::make_index_sequence<size>{},
                                         Vector<T, size>(value)} {}

  template <class... U>
  constexpr Matrix(const Vector<T, size>& first, const U&... next) noexcept
      : RectangularMatrix<T, size, size>{first, next...} {}

  constexpr Matrix(T value) noexcept
      : RectangularMatrix<T, size, size>{std::make_index_sequence<size>(),
                                         value} {}

  template <class U>
  constexpr explicit Matrix(
      const RectangularMatrix<U, size, size>& other) noexcept
      : RectangularMatrix<T, size, size>(other) {}

  template <std::size_t otherSize>
  constexpr explicit Matrix(
      const RectangularMatrix<T, otherSize, otherSize>& other) noexcept
      : Matrix<T, size>{std::make_index_sequence<size>{}, other} {}

  constexpr Matrix(const RectangularMatrix<T, size, size>& other) noexcept
      : RectangularMatrix<T, size, size>(other) {}

 private:
  template <std::size_t otherSize, std::size_t... col>
  constexpr explicit Matrix(
      std::index_sequence<col...>,
      const RectangularMatrix<T, otherSize, otherSize>& other) noexcept
      : RectangularMatrix<T, size, size>{
            valueOrIdentityVector<size, col>(other)...} {}
};

}  // namespace Ajisai::Math

#endif
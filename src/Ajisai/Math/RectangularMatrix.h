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

#ifndef AJISAI_MATH_RECTANGULARMATRIX_H_
#define AJISAI_MATH_RECTANGULARMATRIX_H_

#include <cstddef>

#include <Ajisai/Math/Tags.h>
#include <Ajisai/Math/Vector.h>

namespace Ajisai::Math {

template <class T, std::size_t cols, std::size_t rows>
class RectangularMatrix {
  static_assert(rows != 0 && cols != 0,
                "RectangularMatrix cannot have zero size");

  template <class, std::size_t, std::size_t>
  friend class RectangularMatrix;

 public:
  enum : std::size_t {
    Rows = rows,
    Cols = cols,

    DiagonalSize = (cols < rows ? cols : rows)
  };

  constexpr RectangularMatrix() noexcept : _data{} {}

  constexpr explicit RectangularMatrix(IdentityInitT, T value = T(1)) noexcept
      : RectangularMatrix<T, cols, rows>{
            std::make_index_sequence<DiagonalSize>{},
            Vector<T, DiagonalSize>(value)} {}

  template <class... U>
  constexpr RectangularMatrix(const Vector<T, rows>& first,
                              const U&... next) noexcept
      : _data{first, next...} {
    static_assert(
        sizeof...(next) + 1 == cols,
        "invalid number of arguments passed to Rectangular constructor");
  }

  constexpr explicit RectangularMatrix(T value) noexcept
      : RectangularMatrix{std::make_index_sequence<cols>{}, value} {}

  template <class U>
  constexpr explicit RectangularMatrix(
      const RectangularMatrix<U, cols, rows>& other) noexcept
      : RectangularMatrix{std::make_index_sequence<cols>{}, other} {}

  constexpr RectangularMatrix(
      const RectangularMatrix<T, cols, rows>&) noexcept = default;

  T* data() { return _data[0].data(); }
  constexpr const T* data() const { return _data[0].data(); }

  Vector<T, rows>& operator[](std::size_t col) { return _data[col]; }
  constexpr const Vector<T, rows>& operator[](std::size_t col) const {
    return _data[col];
  }

 private:
  template <class, std::size_t>
  friend class Matrix;

  template <std::size_t... sequence>
  constexpr explicit RectangularMatrix(std::index_sequence<sequence...>,
                                       T value) noexcept
      : _data{Vector<T, rows>{(static_cast<void>(sequence), value)}...} {}

  template <std::size_t... sequence>
  constexpr explicit RectangularMatrix(std::index_sequence<sequence...>,
                                       const Vector<T, DiagonalSize>& diagonal);

  template <class U, std::size_t... sequence>
  constexpr explicit RectangularMatrix(
      std::index_sequence<sequence...>,
      const RectangularMatrix<U, cols, rows>& other) noexcept
      : _data{Vector<T, rows>{other[sequence]}...} {}

  Vector<T, rows> _data[cols];
};

namespace {

template <std::size_t rows, std::size_t i, class T, std::size_t... sequence>
Vector<T, rows> diagonalMatrixColumn2(std::index_sequence<sequence...>,
                                      const T& number) {
  return {(sequence == i ? number : T(0))...};
}

template <std::size_t rows, std::size_t i, class T>
Vector<T, rows> diagonalMatrixColumn(const T& number) {
  return diagonalMatrixColumn2<rows, i, T>(std::make_index_sequence<rows>(),
                                           number);
}

}  // namespace

template <class T, std::size_t cols, std::size_t rows>
template <std::size_t... sequence>
constexpr RectangularMatrix<T, cols, rows>::RectangularMatrix(
    std::index_sequence<sequence...>, const Vector<T, DiagonalSize>& diagonal)
    : _data{diagonalMatrixColumn<rows, sequence>(
          sequence < DiagonalSize ? diagonal[sequence] : T{})...} {}

}  // namespace Ajisai::Math

template <class T, std::size_t cols, std::size_t rows>
std::ostream& operator<<(
    std::ostream& ostream,
    const Ajisai::Math::RectangularMatrix<T, cols, rows>& value) {
  ostream << "Matrix(";
  for (std::size_t row = 0; row != rows; ++row) {
    if (row != 0) ostream << ",\n      ";
    for (std::size_t col = 0; col != cols; ++col) {
      if (col != 0) ostream << ",";
      ostream << value[col][row];
    }
  }
  return ostream << ")";
}

// extern template std::ostream& operator<<(
//     std::ostream& ostream, const Ajisai::Math::RectangularMatrix<float, 3,
//     3>&);
// extern template std::ostream& operator<<(
//     std::ostream& ostream, const Ajisai::Math::RectangularMatrix<float, 4,
//     4>&);

#endif
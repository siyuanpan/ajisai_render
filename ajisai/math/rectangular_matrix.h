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
#include <cstddef>

#include <ajisai/math/tags.h>
#include <ajisai/math/vector.h>

AJ_BEGIN

template <class, std::size_t, std::size_t>
class RectangularMatrix;

namespace {

// template <std::size_t cols, std::size_t rows, std::size_t otherCols,
//           std::size_t otherRows, class T, std::size_t col, std::size_t...
//           row>
// constexpr Vector<T, rows> valueOrIdentityVector(
//     std::index_sequence<rows>,
//     const RectangularMatrix<T, otherCols, otherRows>& other, T value) {
//   return {(col < otherCols && row < otherRows ? other[col][row]
//            : col == row                       ? value
//                                               : T{0})...};
// }

// template <std::size_t cols, std::size_t rows, std::size_t otherCols,
//           std::size_t otherRows, class T, std::size_t col>
// constexpr Vector<T, rows> valueOrIdentityVector(
//     const RectangularMatrix<T, otherCols, otherRows>& other, T value) {
//   return valueOrIdentityVector<cols, rows, otherCols, otherRows, T, col>(
//       std::index_sequence<rows>{}, other, value);
// }

}  // namespace

namespace Impl {
template <class, std::size_t>
struct MatrixDeterminant;
}

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

  // template <std::size_t otherCols, std::size_t otherRows>
  // constexpr explicit RectangularMatrix(
  //     IdentityInitT, const RectangularMatrix<T, otherCols, otherRows>& other,
  //     T value = T(1)) noexcept
  //     : RectangularMatrix<T, cols, rows>{
  //           IdentityInit, std::make_index_sequence<cols>{}, other, value} {}

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

  Vector<T, cols> row(std::size_t row) const;

  void setRow(std::size_t row, const Vector<T, cols>& data);

  RectangularMatrix<T, rows, cols> transposed() const;

  RectangularMatrix<T, cols, rows>& operator/=(T scalar) {
    for (std::size_t i = 0; i != cols; ++i) {
      _data[i] /= scalar;
    }

    return *this;
  }

  RectangularMatrix<T, cols, rows> operator/(T scalar) const {
    return RectangularMatrix<T, cols, rows>(*this) /= scalar;
  }

  template <std::size_t size>
  RectangularMatrix<T, size, rows> operator*(
      const RectangularMatrix<T, size, cols>& other) const;

  Vector<T, rows> operator*(const Vector<T, cols>& other) const {
    return operator*(RectangularMatrix<T, 1, cols>(other))[0];
  }

 private:
  template <class, std::size_t>
  friend class Matrix;
  template <class, std::size_t>
  friend struct Impl::MatrixDeterminant;

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

  // template <std::size_t otherCols, std::size_t otherRows, std::size_t... col>
  // constexpr explicit RectangularMatrix(
  //     IdentityInitT, std::index_sequence<col...>,
  //     const RectangularMatrix<T, otherCols, otherRows>& other, T value)
  //     noexcept : RectangularMatrix<T, cols, rows>{
  //           valueOrIdentityVector<cols, rows, otherCols, otherRows, T, col>(
  //               other, value)...} {}

  Vector<T, rows> _data[cols];
};

template <class T, std::size_t cols, std::size_t rows>
inline RectangularMatrix<T, cols, rows> operator/(
    typename std::common_type<T>::type scalar,
    const RectangularMatrix<T, cols, rows>& matrix) {
  RectangularMatrix<T, cols, rows> out{};

  for (std::size_t i = 0; i != cols; ++i) {
    out[i] = scalar / matrix[i];
  }

  return out;
}

template <class T, std::size_t size, std::size_t cols>
inline RectangularMatrix<T, cols, size> operator*(
    const Vector<T, size>& vector,
    const RectangularMatrix<T, cols, 1>& matrix) {
  return RectangularMatrix<T, 1, size>(vector) * matrix;
}

template <class T, std::size_t cols, std::size_t rows>
template <std::size_t size>
inline RectangularMatrix<T, size, rows>
RectangularMatrix<T, cols, rows>::operator*(
    const RectangularMatrix<T, size, cols>& other) const {
  RectangularMatrix<T, size, rows> out{};

  for (std::size_t col = 0; col != size; ++col)
    for (std::size_t row = 0; row != rows; ++row)
      for (std::size_t pos = 0; pos != cols; ++pos)
        out._data[col]._data[row] +=
            _data[pos]._data[row] * other._data[col]._data[pos];

  return out;
}

template <class T, std::size_t cols, std::size_t rows>
inline Vector<T, cols> RectangularMatrix<T, cols, rows>::row(
    std::size_t row) const {
  Vector<T, cols> out;

  for (std::size_t i = 0; i != cols; ++i) out[i] = _data[i][row];

  return out;
}

template <class T, std::size_t cols, std::size_t rows>
inline void RectangularMatrix<T, cols, rows>::setRow(
    std::size_t row, const Vector<T, cols>& data) {
  for (std::size_t i = 0; i != cols; ++i) _data[i][row] = data[i];
}

#define AJISA_RECTANGULARMATRIX_SUBCLASS_IMPL(cols, rows, ...)      \
  __VA_ARGS__& operator/=(T number) {                               \
    aj::RectangularMatrix<T, cols, rows>::operator/=(number);       \
    return *this;                                                   \
  }                                                                 \
  __VA_ARGS__ operator/(T number) const {                           \
    return aj::RectangularMatrix<T, cols, rows>::operator/(number); \
  }

#define AJISAI_MATRIX_OPERATOR_IMPL(...)                                     \
  template <class T, std::size_t size>                                       \
  inline __VA_ARGS__ operator/(typename std::common_type<T>::type number,    \
                               const __VA_ARGS__& matrix) {                  \
    return number /                                                          \
           static_cast<const aj::RectangularMatrix<T, size, size>&>(matrix); \
  }

#define AJISAI_MATRIXn_OPERATOR_IMPLEMENTATION(size, Type)                   \
  template <class T>                                                         \
  inline Type<T> operator/(typename std::common_type<T>::type number,        \
                           const Type<T>& matrix) {                          \
    return number /                                                          \
           static_cast<const aj::RectangularMatrix<T, size, size>&>(matrix); \
  }

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

template <class T, std::size_t cols, std::size_t rows>
inline RectangularMatrix<T, rows, cols>
RectangularMatrix<T, cols, rows>::transposed() const {
  RectangularMatrix<T, rows, cols> out{};

  for (std::size_t col = 0; col != cols; ++col) {
    for (std::size_t row = 0; row != rows; ++row) {
      out._data[row][col] = _data[col][row];
    }
  }

  return out;
}

AJ_END

template <class T, std::size_t cols, std::size_t rows>
std::ostream& operator<<(std::ostream& ostream,
                         const aj::RectangularMatrix<T, cols, rows>& value) {
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

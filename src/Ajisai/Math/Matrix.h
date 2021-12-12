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

#include <cassert>

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

namespace Impl {
template <class, std::size_t>
struct MatrixDeterminant;
}

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

  //   template <std::size_t otherCols, std::size_t otherRows>
  //   constexpr explicit Matrix(
  //       IdentityInitT, const RectangularMatrix<T, otherCols, otherRows>&
  //       other, T value = T(1)) noexcept : RectangularMatrix<T, size,
  //       size>{IdentityInit, other, value} {}

  template <std::size_t otherSize>
  constexpr explicit Matrix(
      const RectangularMatrix<T, otherSize, otherSize>& other) noexcept
      : Matrix<T, size>{std::make_index_sequence<size>{}, other} {}

  constexpr Matrix(const RectangularMatrix<T, size, size>& other) noexcept
      : RectangularMatrix<T, size, size>(other) {}

  Matrix<T, size> inverted() const;

  Matrix<T, size> adjugate() const;

  Matrix<T, size - 1> ij(std::size_t skipCol, std::size_t skipRow) const;

  T cofactor(std::size_t col, std::size_t row) const;

  bool isOrthogonal() const;

  Matrix<T, size> invertedOrthogonal() const {
    assert(isOrthogonal());
    return RectangularMatrix<T, size, size>::transposed();
  }

  Matrix<T, size> operator*(const Matrix<T, size>& other) const {
    return RectangularMatrix<T, size, size>::operator*(other);
  }

  template <std::size_t otherCols>
  RectangularMatrix<T, otherCols, size> operator*(
      const RectangularMatrix<T, otherCols, size>& other) const {
    return RectangularMatrix<T, size, size>::operator*(other);
  }

  Vector<T, size> operator*(const Vector<T, size>& other) const {
    return RectangularMatrix<T, size, size>::operator*(other);
  }

  Matrix<T, size> transposed() const {
    return RectangularMatrix<T, size, size>::transposed();
  }

  T determinant() const { return Impl::MatrixDeterminant<T, size>()(*this); }

  AJISA_RECTANGULARMATRIX_SUBCLASS_IMPL(size, size, Matrix<T, size>)

 private:
  friend struct Impl::MatrixDeterminant<T, size>;

  template <std::size_t otherSize, std::size_t... col>
  constexpr explicit Matrix(
      std::index_sequence<col...>,
      const RectangularMatrix<T, otherSize, otherSize>& other) noexcept
      : RectangularMatrix<T, size, size>{
            valueOrIdentityVector<size, col>(other)...} {}
};

template <class T>
using Matrix2x2 = Matrix<T, 2>;

template <class T>
using Matrix3x3 = Matrix<T, 3>;

template <class T>
using Matrix4x4 = Matrix<T, 4>;

AJISAI_MATRIX_OPERATOR_IMPL(Matrix<T, size>)

#define AJISAI_MATRIX_SUBCLASS_IMPL(size, Type, VectorType)               \
  VectorType<T>& operator[](std::size_t col) {                            \
    return static_cast<VectorType<T>&>(Matrix<T, size>::operator[](col)); \
  }                                                                       \
  constexpr const VectorType<T> operator[](std::size_t col) const {       \
    return VectorType<T>(Matrix<T, size>::operator[](col));               \
  }                                                                       \
  VectorType<T> row(std::size_t row) const {                              \
    return VectorType<T>(Matrix<T, size>::row(row));                      \
  }                                                                       \
  Type<T> operator*(const Matrix<T, size>& other) const {                 \
    return Matrix<T, size>::operator*(other);                             \
  }                                                                       \
  template <std::size_t otherCols>                                        \
  RectangularMatrix<T, otherCols, size> operator*(                        \
      const RectangularMatrix<T, otherCols, size>& other) const {         \
    return Matrix<T, size>::operator*(other);                             \
  }                                                                       \
                                                                          \
  VectorType<T> operator*(const Vector<T, size>& other) const {           \
    return Matrix<T, size>::operator*(other);                             \
  }                                                                       \
  Type<T> transposed() const { return Matrix<T, size>::transposed(); }    \
  Type<T> invertedOrthogonal() const {                                    \
    return Matrix<T, size>::invertedOrthogonal();                         \
  }

namespace Impl {
template <class T, std::size_t size>
struct MatrixDeterminant {
  T operator()(const Matrix<T, size>& m) {
    T out(0);

    for (std::size_t col = 0; col != size; ++col)
      out += m[col][0] * m.cofactor(col, 0);

    return out;
  }

  T operator()(const Matrix<T, size + 1>& m, const std::size_t skipCol,
               const std::size_t skipRow) {
    return m.ij(skipCol, skipRow).determinant();
  }
};

template <class T>
struct MatrixDeterminant<T, 3> {
  constexpr T operator()(const Matrix<T, 3>& m) const {
    return m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
           m[0][1] * (m[1][0] * m[2][2] - m[2][0] * m[1][2]) +
           m[0][2] * (m[1][0] * m[2][1] - m[2][0] * m[1][1]);
  }

  constexpr T operator()(const Matrix<T, 4>& m, const std::size_t skipCol,
                         const std::size_t skipRow) const {
#define _col(i) _data[i + (i >= skipCol)]
#define _row(i) _data[i + (i >= skipRow)]
    return m._col(0)._row(0) * ((m._col(1)._row(1) * m._col(2)._row(2)) -
                                (m._col(2)._row(1) * m._col(1)._row(2))) -
           m._col(0)._row(1) * (m._col(1)._row(0) * m._col(2)._row(2) -
                                m._col(2)._row(0) * m._col(1)._row(2)) +
           m._col(0)._row(2) * (m._col(1)._row(0) * m._col(2)._row(1) -
                                m._col(2)._row(0) * m._col(1)._row(1));
#undef _col
#undef _row
  }
};

template <class T>
struct MatrixDeterminant<T, 2> {
  constexpr T operator()(const Matrix<T, 2>& m) const {
    return m._data[0]._data[0] * m._data[1]._data[1] -
           m._data[1]._data[0] * m._data[0]._data[1];
  }

  constexpr T operator()(const Matrix<T, 3>& m, const std::size_t skipCol,
                         const std::size_t skipRow) const {
#define _col(i) _data[i + (i >= skipCol)]
#define _row(i) _data[i + (i >= skipRow)]
    return m._col(0)._row(0) * m._col(1)._row(1) -
           m._col(1)._row(0) * m._col(0)._row(1);
#undef _col
#undef _row
  }
};

template <class T>
struct MatrixDeterminant<T, 1> {
  constexpr T operator()(const Matrix<T, 1>& m) const {
    return m._data[0]._data[0];
  }

  constexpr T operator()(const Matrix<T, 2>& m, const std::size_t skipCol,
                         const std::size_t skipRow) const {
    return m._data[0 + (0 >= skipCol)]._data[0 + (0 >= skipRow)];
  }
};

}  // namespace Impl

template <class T, std::size_t size>
bool Matrix<T, size>::isOrthogonal() const {
  for (std::size_t i = 0; i != size; ++i)
    if (!RectangularMatrix<T, size, size>::_data[i].isNormalized())
      return false;

  for (std::size_t i = 0; i != size - 1; ++i)
    for (std::size_t j = i + 1; j != size; ++j)
      if (dot(RectangularMatrix<T, size, size>::_data[i],
              RectangularMatrix<T, size, size>::_data[j]) >
          std::numeric_limits<T>::epsilon())
        return false;

  return true;
}

template <class T, std::size_t size>
Matrix<T, size - 1> Matrix<T, size>::ij(const std::size_t skipCol,
                                        const std::size_t skipRow) const {
  Matrix<T, size - 1> out{};

  for (std::size_t col = 0; col != size - 1; ++col)
    for (std::size_t row = 0; row != size - 1; ++row)
      out[col][row] =
          RectangularMatrix<T, size, size>::_data[col + (col >= skipCol)]
                                                 [row + (row >= skipRow)];

  return out;
}

template <class T, std::size_t size>
T Matrix<T, size>::cofactor(std::size_t col, std::size_t row) const {
  return (((row + col) & 1) ? -1 : 1) *
         Impl::MatrixDeterminant<T, size - 1>()(*this, col, row);
}

template <class T, std::size_t size>
Matrix<T, size> Matrix<T, size>::adjugate() const {
  Matrix<T, size> out{};

  for (std::size_t col = 0; col != size; ++col)
    for (std::size_t row = 0; row != size; ++row)
      out[col][row] = cofactor(row, col);

  return out;
}

template <class T, std::size_t size>
Matrix<T, size> Matrix<T, size>::inverted() const {
  return adjugate() / determinant();
}

}  // namespace Ajisai::Math

#endif
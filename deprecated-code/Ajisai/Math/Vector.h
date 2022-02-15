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

#ifndef AJISAI_MATH_VECTOR_H_
#define AJISAI_MATH_VECTOR_H_

#include <cmath>
#include <iostream>
#include <type_traits>
#include <utility>

#include "Ajisai/Math/BoolVector.h"
#include "Ajisai/Math/TypeTraits.h"

namespace Ajisai::Math {

template <class T>
constexpr typename std::enable_if<std::is_scalar<T>::value, T>::type min(T a,
                                                                         T b) {
  return b < a ? b : a;
}

template <class T>
constexpr typename std::enable_if<std::is_scalar<T>::value, T>::type max(T a,
                                                                         T b) {
  return a < b ? b : a;
}

namespace Impl {
template <class T>
constexpr T repeat(T value, std::size_t) {
  return value;
}
}  // namespace Impl

namespace Impl {
template <class, std::size_t>
struct MatrixDeterminant;
}

template <class, std::size_t>
class Vector;

template <class U, std::size_t size_>
inline U dot(const Vector<U, size_>& a, const Vector<U, size_>& b) {
  U out{};
  for (std::size_t i = 0; i != size_; ++i) {
    out += a._data[i] * b._data[i];
  }
  return out;
}

template <class T, std::size_t size>
class Vector {
  static_assert(size != 0, "Vector cannot have zero size");

 public:
  //   typedef T Type;

  //   enum : std::size_t { Size = size };

  static Vector<T, size>& from(T* data) {
    return *reinterpret_cast<Vector<T, size>*>(data);
  }

  static const Vector<T, size>& from(const T* data) {
    return *reinterpret_cast<const Vector<T, size>*>(data);
  }

  constexpr Vector() noexcept : _data{} {}

  template <class... U, class V = typename std::enable_if<
                            sizeof...(U) + 1 == size, T>::type>
  constexpr Vector(T first, U... next) noexcept : _data{first, next...} {}

  template <class U, class V = typename std::enable_if<
                         std::is_same<T, U>::value && size != 1, T>::type>
  constexpr explicit Vector(U value) noexcept
      : Vector(std::make_index_sequence<size>{}, value) {}

  template <class U>
  constexpr explicit Vector(const Vector<U, size>& other) noexcept
      : Vector{std::make_index_sequence<size>{}, other} {}

  constexpr Vector(const Vector<T, size>&) noexcept = default;

  T* data() { return _data; }
  constexpr const T* data() const { return _data; }

  T& operator[](std::size_t pos) { return _data[pos]; }
  constexpr T operator[](std::size_t pos) const { return _data[pos]; }

  bool operator==(const Vector<T, size>& other) const {
    for (std::size_t i = 0; i != size; ++i)
      if (_data[i] != other._data[i]) return false;

    return true;
  }

  bool operator!=(const Vector<T, size>& other) const {
    return !operator==(other);
  }

  BoolVector<size> operator<(const Vector<T, size>& other) const;
  BoolVector<size> operator<=(const Vector<T, size>& other) const;
  BoolVector<size> operator>(const Vector<T, size>& other) const;
  BoolVector<size> operator>=(const Vector<T, size>& other) const;

  template <class U = T>
  typename std::enable_if<std::is_signed<U>::value, Vector<T, size>>::type
  operator-() const;

  Vector<T, size>& operator+=(const Vector<T, size>& other) {
    for (std::size_t i = 0; i != size; ++i) {
      _data[i] += other._data[i];
    }

    return *this;
  }

  Vector<T, size> operator+(const Vector<T, size>& other) const {
    return Vector<T, size>(*this) += other;
  }

  Vector<T, size>& operator-=(const Vector<T, size>& other) {
    for (std::size_t i = 0; i != size; ++i) {
      _data[i] -= other._data[i];
    }

    return *this;
  }

  Vector<T, size> operator-(const Vector<T, size>& other) const {
    return Vector<T, size>(*this) -= other;
  }

  Vector<T, size> operator*=(const Vector<T, size>& other) {
    for (std::size_t i = 0; i != size; ++i) {
      _data[i] *= other._data[i];
    }

    return *this;
  }

  Vector<T, size> operator*(const Vector<T, size>& other) const {
    return Vector<T, size>(*this) *= other;
  }

  Vector<T, size>& operator/=(const Vector<T, size>& other) {
    for (std::size_t i = 0; i != size; ++i) {
      _data[i] /= other._data[i];
    }

    return *this;
  }

  Vector<T, size> operator/(const Vector<T, size>& other) const {
    return Vector<T, size>(*this) /= other;
  }

  Vector<T, size>& operator*=(T scalar) {
    for (std::size_t i = 0; i != size; ++i) _data[i] *= scalar;

    return *this;
  }

  Vector<T, size> operator*(T scalar) const {
    return Vector<T, size>(*this) *= scalar;
  }

  Vector<T, size> operator/=(T scalar) {
    for (std::size_t i = 0; i != size; ++i) _data[i] /= scalar;

    return *this;
  }

  Vector<T, size> operator/(T scalar) const {
    return Vector<T, size>(*this) /= scalar;
  }

  T dot() const { return Math::dot(*this, *this); }

  T length() const { return T(std::sqrt(dot())); }

  bool isNormalized() const { return isNormalizedSquared(dot()); }

  template <class U = T>
  typename std::enable_if<std::is_floating_point<U>::value, T>::type invLength()
      const {
    return T(1) / length();
  }

  template <class U = T>
  typename std::enable_if<std::is_floating_point<U>::value,
                          Vector<T, size>>::type
  normalized() const {
    return (*this) * invLength();
  }

  T min() const;

  T max() const;

  std::pair<T, T> minmax() const;

 protected:
  T _data[size];

 private:
  template <class, std::size_t>
  friend class Matrix;
  template <class, std::size_t>
  friend struct Impl::MatrixDeterminant;
  template <class, std::size_t, std::size_t>
  friend class RectangularMatrix;

  template <class U, std::size_t size_>
  friend U dot(const Vector<U, size_>&, const Vector<U, size_>&);

  template <std::size_t... sequence>
  constexpr explicit Vector(std::index_sequence<sequence...>, T value) noexcept
      : _data{Impl::repeat(value, sequence)...} {}

  template <class U, std::size_t... sequence>
  constexpr explicit Vector(std::index_sequence<sequence...>,
                            const Vector<U, size>& other) noexcept
      : _data{T(other[sequence])...} {}
};

template <class T, std::size_t size>
inline Vector<T, size> operator*(typename std::common_type<T>::type scalar,
                                 const Vector<T, size>& vector) {
  return vector * scalar;
}

template <class T, std::size_t size>
inline Vector<T, size> operator/(typename std::common_type<T>::type scalar,
                                 const Vector<T, size>& vector) {
  Vector<T, size> out;

  for (std::size_t i = 0; i != size; ++i) {
    out[i] = scalar / vector[i];
  }

  return out;
}

#define VECTOR_SUBCLASS_OPERATOR_IMPL(Type, size)                              \
  static Type<T>& from(T* data) { return *reinterpret_cast<Type<T>*>(data); }  \
  static const Type<T>& from(const T* data) {                                  \
    return *reinterpret_cast<const Type<T>*>(data);                            \
  }                                                                            \
  template <class U = T>                                                       \
  typename std::enable_if<std::is_signed<U>::value, Type<T>>::type operator-() \
      const {                                                                  \
    return Vector<T, size>::operator-();                                       \
  }                                                                            \
  Type<T>& operator+=(const Vector<T, size>& other) {                          \
    Vector<T, size>::operator+=(other);                                        \
    return *this;                                                              \
  }                                                                            \
  Type<T> operator+(const Vector<T, size>& other) const {                      \
    return Vector<T, size>::operator+(other);                                  \
  }                                                                            \
  Type<T>& operator-=(const Vector<T, size>& other) {                          \
    Vector<T, size>::operator-=(other);                                        \
    return *this;                                                              \
  }                                                                            \
  Type<T> operator-(const Vector<T, size>& other) const {                      \
    return Vector<T, size>::operator-(other);                                  \
  }                                                                            \
  Type<T>& operator*=(const Vector<T, size>& other) const {                    \
    Vector<T, size>::operator*=(other);                                        \
    return *this;                                                              \
  }                                                                            \
  Type<T> operator*(const Vector<T, size>& other) const {                      \
    return Vector<T, size>::operator*(other);                                  \
  }                                                                            \
  Type<T>& operator*=(T scalar) {                                              \
    Vector<T, size>::operator*=(scalar);                                       \
    return *this;                                                              \
  }                                                                            \
  Type<T> operator*(T scalar) const {                                          \
    return Vector<T, size>::operator*(scalar);                                 \
  }                                                                            \
  template <class U = T>                                                       \
  typename std::enable_if<std::is_floating_point<U>::value, Type<T>>::type     \
  normalized() const {                                                         \
    return Vector<T, size>::normalized();                                      \
  }

#define VECTOR_FUNCTION_IMPL(Type, size)                               \
  template <class T>                                                   \
  inline Type<T> operator*(typename std::common_type<T>::type number,  \
                           const Type<T>& vector) {                    \
    return number * static_cast<const Vector<T, size>&>(vector);       \
  }                                                                    \
  template <class T>                                                   \
  inline Type<T> operator/(typename std::common_type<T>::type number,  \
                           const Type<T>& vector) {                    \
    return number / static_cast<const Math::Vector<T, size>&>(vector); \
  }

template <class T, std::size_t size>
inline BoolVector<size> Vector<T, size>::operator<(
    const Vector<T, size>& other) const {
  BoolVector<size> out;

  for (std::size_t i = 0; i != size; ++i) {
    out.set(i, _data[i] < other._data[i]);
  }

  return out;
}
template <class T, std::size_t size>
inline BoolVector<size> Vector<T, size>::operator<=(
    const Vector<T, size>& other) const {
  BoolVector<size> out;

  for (std::size_t i = 0; i != size; ++i) {
    out.set(i, _data[i] <= other._data[i]);
  }

  return out;
}

template <class T, std::size_t size>
inline BoolVector<size> Vector<T, size>::operator>=(
    const Vector<T, size>& other) const {
  BoolVector<size> out;

  for (std::size_t i = 0; i != size; ++i) {
    out.set(i, _data[i] >= other._data[i]);
  }

  return out;
}

template <class T, std::size_t size>
inline BoolVector<size> Vector<T, size>::operator>(
    const Vector<T, size>& other) const {
  BoolVector<size> out;

  for (std::size_t i = 0; i != size; ++i) {
    out.set(i, _data[i] > other._data[i]);
  }

  return out;
}

template <class T, std::size_t size>
template <class U>
inline typename std::enable_if<std::is_signed<U>::value, Vector<T, size>>::type
Vector<T, size>::operator-() const {
  Vector<T, size> out;
  for (std::size_t i = 0; i != size; ++i) {
    out._data[i] = -_data[i];
  }

  return out;
}

template <class T, std::size_t size>
inline T Vector<T, size>::min() const {
  T out(_data[0]);

  for (std::size_t i = 1; i != size; ++i) {
    out = Math::min(out, _data[i]);
  }

  return out;
}

template <class T, std::size_t size>
inline T Vector<T, size>::max() const {
  T out(_data[0]);

  for (std::size_t i = 1; i != size; ++i) {
    out = Math::max(out, _data[i]);
  }

  return out;
}

}  // namespace Ajisai::Math

template <class T, std::size_t size>
std::ostream& operator<<(std::ostream& ostream,
                         const Ajisai::Math::Vector<T, size>& value) {
  ostream << "Vector(";
  for (std::size_t i = 0; i != size; ++i) {
    if (i != 0) ostream << ",";
    ostream << value[i];
  }
  return ostream << ")";
}

#endif
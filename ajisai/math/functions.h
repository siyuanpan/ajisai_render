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

#include <algorithm>
#include <cmath>
#include <type_traits>

#include <ajisai/math/vector.h>

AJ_BEGIN

template <class T, std::size_t size>
inline Vector<T, size> sqrt(const Vector<T, size>& a) {
  Vector<T, size> out{};
  for (std::size_t i = 0; i != size; ++i) out[i] = std::sqrt(a[i]);
  return out;
}

template <class T>
inline typename std::enable_if<std::is_scalar<T>::value, T>::type pow(
    T base, T exponent) {
  return std::pow(base, exponent);
}

template <class T, std::size_t size>
inline Vector<T, size> pow(const Vector<T, size>& base, T exponent) {
  Vector<T, size> out;
  for (std::size_t i = 0; i != size; ++i) {
    out[i] = pow(base[i], exponent);
  }
  return out;
}

template <class T, std::size_t size>
inline Vector<T, size> clamp(const Vector<T, size>& value, T min, T max) {
  Vector<T, size> out;
  for (std::size_t i = 0; i != size; ++i) {
    out[i] = std::clamp(value[i], min, max);
  }
  return out;
}

template <class T, std::size_t size>
inline Vector<T, size> clamp(const Vector<T, size>& value,
                             const Vector<T, size>& min,
                             const Vector<T, size>& max) {
  Vector<T, size> out;
  for (std::size_t i = 0; i != size; ++i) {
    out[i] = std::clamp(value[i], min[i], max[i]);
  }
  return out;
}

template <class T, std::size_t size>
inline Vector<T, size> lerp(const Vector<T, size>& a, const Vector<T, size>& b,
                            const BoolVector<size>& t) {
  Vector<T, size> out;
  for (std::size_t i = 0; i != size; ++i) {
    out[i] = t[i] ? b[i] : a[i];
  }

  return out;
}

template <class T>
constexpr typename std::enable_if<std::is_scalar<T>::value, T>::type min(T a,
                                                                         T b);

template <class T, std::size_t size>
inline Vector<T, size> min(const Vector<T, size>& a, const Vector<T, size>& b) {
  Vector<T, size> out;
  for (std::size_t i = 0; i != size; ++i) {
    out[i] = Math::min(a[i], b[i]);
  }
  return out;
}

template <class T>
constexpr typename std::enable_if<std::is_scalar<T>::value, T>::type max(T a,
                                                                         T b);

template <class T, std::size_t size>
inline Vector<T, size> max(const Vector<T, size>& a, const Vector<T, size>& b) {
  Vector<T, size> out;
  for (std::size_t i = 0; i != size; ++i) {
    out[i] = Math::max(a[i], b[i]);
  }
  return out;
}

AJ_END
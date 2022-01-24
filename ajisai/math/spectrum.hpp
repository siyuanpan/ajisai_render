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
#include <initializer_list>

#include <ajisai/math/vector.h>

AJ_BEGIN
template <std::size_t size>
class CoefficientSpectrum : public Vector<float, size> {
 public:
  constexpr CoefficientSpectrum() noexcept : Vector<float, size>{} {}
  constexpr explicit CoefficientSpectrum(float value) noexcept
      : Vector<float, size>(value) {}

  constexpr CoefficientSpectrum(const CoefficientSpectrum<size>& other) noexcept
      : Vector<float, size>(other) {}

  constexpr CoefficientSpectrum(const Vector<float, size>& other) noexcept
      : Vector<float, size>(other) {}

  template <class... U, class V = typename std::enable_if<
                            sizeof...(U) + 1 == size, float>::type>
  constexpr CoefficientSpectrum(float first, U... next) noexcept
      : Vector<float, size>{first, next...} {}

  CoefficientSpectrum<size>& operator*=(const Vector<float, size>& other) {
    Vector<float, size>::operator*=(other);
    return *this;
  }

  CoefficientSpectrum<size> operator*(const Vector<float, size>& other) const {
    return Vector<float, size>::operator*(other);
  }

  CoefficientSpectrum<size>& operator*=(float scalar) {
    Vector<float, size>::operator*=(scalar);
    return *this;
  }

  CoefficientSpectrum<size> operator*(float scalar) const {
    return Vector<float, size>::operator*(scalar);
  }

  bool isBlack() const { return this->max() <= 0; }
  [[nodiscard]] float luminance() const {
    return 0.2126 * (*this)[0] + 0.7152 * (*this)[1] + 0.0722 * (*this)[2];
  }
  [[nodiscard]] CoefficientSpectrum<size> removeNaN() const {
    CoefficientSpectrum<size> tmp;
    for (size_t i = 0; i < size; i++) {
      auto x = (*this)[i];
      if (std::isnan(x)) {
        tmp[i] = 0;
      } else {
        tmp[i] = x;
      }
    }
    return tmp;
  }
};

AJ_END
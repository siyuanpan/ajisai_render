#ifndef AJISAI_MATH_UNIT_H_
#define AJISAI_MATH_UNIT_H_

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

#include <type_traits>

namespace Ajisai::Math {

template <template <class> class Derived, class T>
class Unit {
  template <template <class> class, class>
  friend class Unit;

 public:
  constexpr Unit() noexcept : _value(T(0)) {}

  constexpr explicit Unit(T value) noexcept : _value(value) {}

  template <class U>
  constexpr explicit Unit(Unit<Derived, U> value) noexcept
      : _value(T(value._value)) {}

  constexpr Unit(const Unit<Derived, T>& other) noexcept = default;

  constexpr explicit operator T() const { return _value; }

  constexpr bool operator==(Unit<Derived, T> other) const {
    return _value == other._value;
  }

  constexpr bool operator!=(Unit<Derived, T> other) const {
    return !operator==(other);
  }

  constexpr bool operator<(Unit<Derived, T> other) const {
    return _value < other._value;
  }

  constexpr bool operator>(Unit<Derived, T> other) const {
    return _value > other._value;
  }

  constexpr bool operator<=(Unit<Derived, T> other) const {
    return !operator>(other);
  }

  constexpr bool operator>=(Unit<Derived, T> other) const {
    return !operator<(other);
  }

  constexpr Unit<Derived, T> operator-() const {
    return Unit<Derived, T>(-_value);
  }

  Unit<Derived, T>& operator+=(Unit<Derived, T> other) {
    _value += other._value;
    return *this;
  }

  constexpr Unit<Derived, T> operator+(Unit<Derived, T> other) const {
    return Unit<Derived, T>(_value + other._value);
  }

  Unit<Derived, T>& operator-=(Unit<Derived, T> other) {
    _value -= other._value;
    return *this;
  }

  constexpr Unit<Derived, T> operator-(Unit<Derived, T> other) const {
    return Unit<Derived, T>(_value - other._value);
  }

  Unit<Derived, T>& operator*=(T number) {
    _value *= number;
    return *this;
  }

  constexpr Unit<Derived, T> operator*(T number) const {
    return Unit<Derived, T>(_value * number);
  }

  Unit<Derived, T>& operator/=(T number) {
    _value /= number;
    return *this;
  }

  constexpr Unit<Derived, T> operator/(T number) const {
    return Unit<Derived, T>(_value / number);
  }

  constexpr T operator/(Unit<Derived, T> other) const {
    return _value / other._value;
  }

 private:
  T _value;
};

template <template <class> class Derived, class T>
constexpr Unit<Derived, T> operator*(typename std::common_type<T>::type number,
                                     const Unit<Derived, T>& value) {
  return value * number;
}

}  // namespace Ajisai::Math

#endif
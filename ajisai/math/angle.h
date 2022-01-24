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
#include <ajisai/math/constants.h>
#include <ajisai/math/unit.h>

#include <iostream>

AJ_BEGIN

template <class>
class Rad;

template <class T>
class Deg : public Unit<Deg, T> {
 public:
  constexpr Deg() noexcept : Unit<Deg, T>() {}

  constexpr explicit Deg(T value) noexcept : Unit<Deg, T>(value) {}

  template <class U>
  constexpr explicit Deg(Unit<Deg, U> value) noexcept : Unit<Deg, T>(value) {}

  constexpr Deg(Unit<Deg, T> value) noexcept : Unit<Deg, T>(value) {}

  constexpr Deg(Unit<Rad, T> value);
};

template <class T>
class Rad : public Unit<Rad, T> {
 public:
  constexpr Rad() noexcept : Unit<Rad, T>{} {}

  constexpr explicit Rad(T value) noexcept : Unit<Rad, T>(value) {}

  template <class U>
  constexpr explicit Rad(Unit<Rad, U> value) noexcept : Unit<Rad, T>(value) {}

  constexpr Rad(Unit<Rad, T> value) noexcept : Unit<Rad, T>(value) {}

  constexpr Rad(Unit<Deg, T> value);
};

constexpr Deg<double> operator"" _deg(long double value) {
  return Deg<double>(double(value));
}

constexpr Deg<float> operator"" _degf(long double value) {
  return Deg<float>(float(value));
}

constexpr Rad<double> operator"" _rad(long double value) {
  return Rad<double>(double(value));
}

constexpr Rad<float> operator"" _radf(long double value) {
  return Rad<float>(float(value));
}

template <class T>
constexpr Deg<T>::Deg(Unit<Rad, T> value)
    : Unit<Deg, T>(T(180) * T(value) / Constants<T>::pi()) {}
template <class T>
constexpr Rad<T>::Rad(Unit<Deg, T> value)
    : Unit<Rad, T>(T(value) * Constants<T>::pi() / T(180)) {}

AJ_END

template <class T, std::size_t size>
std::ostream& operator<<(std::ostream& ostream,
                         const aj::Unit<aj::Deg, T>& value) {
  return ostream << "Deg(" << T(value) << std::endl;
}

template <class T, std::size_t size>
std::ostream& operator<<(std::ostream& ostream,
                         const aj::Unit<aj::Rad, T>& value) {
  return ostream << "Rad(" << T(value) << std::endl;
}

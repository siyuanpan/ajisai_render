#ifndef AJISAI_MATH_BOUNDS_H_
#define AJISAI_MATH_BOUNDS_H_
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

#include "Ajisai/Math/Vector2.h"

namespace Ajisai::Math {

namespace {
template <class, std::size_t>
struct BoundsTraits;

template <class T>
struct BoundsTraits<T, 1> {
  typedef T Type;
  constexpr static Type from(const Vector<T, 1>& value) { return value[0]; }
};

template <class T>
struct BoundsTraits<T, 2> {
  typedef Vector2<T> Type;
  constexpr static Type from(const Vector<T, 2>& value) { return value; }
};
}  // namespace

template <class T, std::size_t size>
class Bounds {
 public:
  typedef typename BoundsTraits<T, size>::Type VectorType;
  // constexpr Bounds() noexcept: Bounds<T, size>{typename std::conditional<size
  // == 1, >::type{}} {}
  constexpr Bounds() noexcept {}

  constexpr Bounds(const VectorType& min, const VectorType& max) noexcept
      : _min{min}, _max{max} {}

  constexpr Bounds(const Bounds<T, size>&) noexcept = default;

  VectorType& min() { return _min; }
  constexpr const VectorType min() const { return _min; }

  VectorType& max() { return _max; }
  constexpr const VectorType max() const { return _max; }

  constexpr const VectorType Size() const { return _max - _min; }

 private:
  //   constexpr explicit Bounds()
  VectorType _min, _max;
};

}  // namespace Ajisai::Math

#endif
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

#include <ostream>
#include <type_traits>

#include <Ajisai/Math/Functions.h>
#include "Ajisai/Math/Vector3.h"

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

template <class T>
struct BoundsTraits<T, 3> {
  typedef Vector3<T> Type;
  constexpr static Type from(const Vector<T, 3>& value) { return value; }
};

}  // namespace

template <class T, std::size_t dimensions>
class Bounds {
 public:
  typedef typename BoundsTraits<T, dimensions>::Type VectorType;
  // constexpr Bounds() noexcept: Bounds<T, size>{typename std::conditional<size
  // == 1, >::type{}} {}
  constexpr Bounds() noexcept {}

  constexpr Bounds(const VectorType& min, const VectorType& max) noexcept
      : _min{min}, _max{max} {}

  constexpr Bounds(const VectorType& min) noexcept : Bounds{min, min} {}

  // constexpr Bounds(const Vector<T, dimensions>& min,
  //                  const Vector<T, dimensions>& max) noexcept
  //     : _min{min}, _max{max} {}

  Bounds(const std::pair<Vector<T, dimensions>, Vector<T, dimensions>>&
             minmax) noexcept
      : _min{minmax.first}, _max{minmax.second} {}

  constexpr Bounds(const Bounds<T, dimensions>&) noexcept = default;

  VectorType& min() { return _min; }
  constexpr const VectorType min() const { return _min; }

  VectorType& max() { return _max; }
  constexpr const VectorType max() const { return _max; }

  VectorType size() const { return _max - _min; }

  VectorType center() const { return (_max + _min) / T(2); }

  template <std::size_t dim = dimensions,
            class = typename std::enable_if<dim == 3>::type>
  T area() const {
    auto d = size();
    return T(2) * (d.x() * d.y() + d.x() * d.z() + d.y() * d.z());
  }

 private:
  //   constexpr explicit Bounds()
  VectorType _min, _max;
};

template <class T, std::size_t dim>
inline Bounds<T, dim> join(const Bounds<T, dim>& a, const Bounds<T, dim>& b) {
  if (a.min() == a.max()) return b;
  if (b.min() == b.max()) return a;
  return {Math::min(a.min(), b.min()), Math::max(a.max(), b.max())};
}

template <class T, std::size_t dim>
inline bool intersects(const Bounds<T, dim>& a, const Bounds<T, dim>& b) {
  return (a.max() > b.min()).all() && (a.min() < b.max()).all();
}

template <class T, std::size_t dim>
inline Bounds<T, dim> intersect(const Bounds<T, dim>& a,
                                const Bounds<T, dim>& b) {
  if (!intersects(a, b)) return {};
  return {Math::max(a.min(), b.min()), Math::min(a.max(), b.max())};
}

}  // namespace Ajisai::Math

template <class T, std::size_t dim>
std::ostream& operator<<(std::ostream& ostream,
                         const Ajisai::Math::Bounds<T, dim>& value) {
  return ostream << "Bounds(" << value.min() << "," << value.max() << ")";
}

#endif
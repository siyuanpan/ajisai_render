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

#ifndef AJISAI_MATH_MATH_H_
#define AJISAI_MATH_MATH_H_

#include <cstddef>

#include "Ajisai/Math/Bounds.hpp"
#include "Ajisai/Math/Color.h"
#include "Ajisai/Math/Constants.h"
#include "Ajisai/Math/Functions.h"
#include "Ajisai/Math/Spectrum.hpp"
#include "Ajisai/Math/Vector2.h"
#include "Ajisai/Math/Vector3.h"
#include "Ajisai/Math/Vector4.h"

namespace Ajisai::Math {
template <class, std::size_t>
class Vector;
template <class>
class Vector2;
template <class>
class Vector3;
template <class>
class Vector4;
template <class>
class Color3;
template <std::size_t>
class CoefficientSpectrum;

template <class, std::size_t>
class Bounds;

typedef Vector2<int> Vector2i;
typedef Vector2<float> Vector2f;
typedef Vector2<double> Vector2d;

typedef Vector3<int> Vector3i;
typedef Vector3<float> Vector3f;

typedef Vector4<float> Vector4f;

typedef CoefficientSpectrum<3> RGBSpectrum;
// typedef Vector3<float> RGBSpectrum;
typedef RGBSpectrum Spectrum;

typedef Bounds<int, 2> Bounds2i;

}  // namespace Ajisai::Math

#endif
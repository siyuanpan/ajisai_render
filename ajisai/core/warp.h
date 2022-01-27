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
#include <ajisai/ajisai.h>
#include <ajisai/math/vector3.h>
#include <ajisai/math/constants.h>

AJ_BEGIN

inline Vector2f squareToUniformDiskConcentric(const Vector2f& sample) {
  auto offset = 2.f * sample - Vector2f(1.f);

  float r, phi;
  if (offset.x() == 0 && offset.y() == 0) {
    return {0, 0};
  } else if (std::abs(offset.x()) > std::abs(offset.y())) {
    r = offset.x();
    phi = Constants<float>::pi4() * (offset.y() / offset.x());
  } else {
    r = offset.y();
    phi = Constants<float>::pi2() -
          Constants<float>::pi4() * (offset.x() / offset.y());
  }

  return {r * std::cos(phi), r * std::sin(phi)};
}

inline Vector3f squareToCosineHemisphere(const Vector2f& sample) {
  auto p = squareToUniformDiskConcentric(sample);
  auto r = dot(p, p);
  auto z = std::sqrt(std::max(0.0f, 1 - r));

  if (z == 0) {
    z = 1e-10f;
  }

  return {p.x(), p.y(), z};
}

inline std::tuple<Vector3f, float> SquareToCosineHemisphereSample(
    const Vector2f& sample) {
  auto p = squareToUniformDiskConcentric(sample);
  auto r = dot(p, p);
  auto z = std::sqrt(std::max(0.0f, 1 - r));

  if (z == 0) {
    z = 1e-10f;
  }

  constexpr float inv_pi = 1.f / Constants<float>::pi();

  return {{p.x(), p.y(), z}, z * inv_pi};
}

AJ_END
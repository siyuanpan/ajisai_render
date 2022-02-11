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

inline Vector3f UniformSampleCone(const Vector2f& u, float cos_theta_max) {
  float cos_theta = (1.f - u[0]) + u[0] * cos_theta_max;
  float sin_theta = std::sqrt(std::max(0.f, 1.f - cos_theta * cos_theta));
  float phi = u[1] * 2 * Constants<float>::pi();
  return Vector3f{std::cos(phi) * sin_theta, std::sin(phi) * sin_theta,
                  cos_theta};
}

inline Vector3f UniformSampleCone(const Vector2f& u, float cos_theta_max,
                                  const Vector3f& x, const Vector3f& y,
                                  const Vector3f& z) {
  float cos_theta = std::lerp(cos_theta_max, 1.f, u[0]);
  float sin_theta = std::sqrt(std::max(0.f, 1.f - cos_theta * cos_theta));
  float phi = u[1] * 2 * Constants<float>::pi();
  return std::cos(phi) * sin_theta * x + std::sin(phi) * sin_theta * y +
         cos_theta * z;
}

inline float UniformConePDF(float cos_theta_max) {
  if (cos_theta_max == 1.0f) return 1.0f;

  return 1.0f / (2.0f * Constants<float>::pi() * (1.0f - cos_theta_max));
}

AJ_END
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

AJ_BEGIN

static constexpr float kDiracAcceptanceThreshold = 1e-3f;

static inline bool checkReflectionConstraint(const Vector3f &wi,
                                             const Vector3f &wo) {
  return std::abs(wi.z() * wo.z() - wi.x() * wo.x() - wi.y() * wo.y() - 1.0f) <
         kDiracAcceptanceThreshold;
}

namespace Fresnel {

static inline float dielectricReflectance(float eta, float cosThetaI,
                                          float &cosThetaT) {
  if (cosThetaI < 0.0f) {
    eta = 1.0f / eta;
    cosThetaI = -cosThetaI;
  }
  float sinThetaTSq = eta * eta * (1.0f - cosThetaI * cosThetaI);
  if (sinThetaTSq > 1.0f) {
    cosThetaT = 0.0f;
    return 1.0f;
  }
  cosThetaT = std::sqrt(std::max(1.0f - sinThetaTSq, 0.0f));

  float Rs = (eta * cosThetaI - cosThetaT) / (eta * cosThetaI + cosThetaT);
  float Rp = (eta * cosThetaT - cosThetaI) / (eta * cosThetaT + cosThetaI);

  return (Rs * Rs + Rp * Rp) * 0.5f;
}

static inline float dielectricReflectance(float eta, float cosThetaI) {
  float cosThetaT;
  return dielectricReflectance(eta, cosThetaI, cosThetaT);
}

// Computes hemispherical integral of dielectricReflectance(ior,
// cos(theta))*cos(theta)
static inline float computeDiffuseFresnel(float ior, const int sampleCount) {
  double diffuseFresnel = 0.0;
  float fb = Fresnel::dielectricReflectance(ior, 0.0f);
  for (int i = 1; i <= sampleCount; ++i) {
    float cosThetaSq = float(i) / sampleCount;
    float fa = Fresnel::dielectricReflectance(
        ior, std::min(std::sqrt(cosThetaSq), 1.0f));
    diffuseFresnel += double(fa + fb) * (0.5 / sampleCount);
    fb = fa;
  }

  return float(diffuseFresnel);
}

}  // namespace Fresnel

AJ_END
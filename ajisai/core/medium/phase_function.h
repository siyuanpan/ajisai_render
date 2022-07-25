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
#include <ajisai/core/bsdf/bsdf.h>
#include <ajisai/core/bsdf/bsdf.h>
#include <ajisai/math/constants.h>
#include <ajisai/core/coordinate_system.h>

AJ_BEGIN

class PhaseFunction {
 public:
  virtual BSDFSampleResult Sample(const Vector3f& wo, TransMode mode,
                                  const Vector3f& sam,
                                  uint8_t type) const noexcept = 0;

  virtual BSDFSampleResult SampleAll(const Vector3f& wo, TransMode mode,
                                     const Vector3f& sam) const noexcept {
    return Sample(wo, mode, sam, kBSDFAll);
  }

  virtual Spectrum EvalAll(const Vector3f& wi, const Vector3f& wo,
                           TransMode mode) const noexcept {
    return Eval(wi, wo, mode, kBSDFAll);
  }

  virtual Spectrum Eval(const Vector3f& wi, const Vector3f& wo, TransMode mode,
                        uint8_t type) const noexcept = 0;

  virtual float PdfAll(const Vector3f& wi, const Vector3f& wo) const noexcept {
    return Pdf(wi, wo, kBSDFAll);
  }

  virtual float Pdf(const Vector3f& wi, const Vector3f& wo,
                    uint8_t type) const noexcept = 0;

  virtual ~PhaseFunction() = default;
};

class HenyeyGreensteinPhaseFunction : public PhaseFunction {
 public:
  HenyeyGreensteinPhaseFunction(float g, const Spectrum& albedo)
      : g_{g}, albedo_{albedo} {}

  virtual Spectrum Eval(const Vector3f& wi, const Vector3f& wo, TransMode mode,
                        uint8_t type) const noexcept override {
    const float u = -dot(wi, wo);
    return Spectrum{phase_func(u)};
  }

  virtual BSDFSampleResult Sample(const Vector3f& wo, TransMode mode,
                                  const Vector3f& sam,
                                  uint8_t type) const noexcept override {
    const float s = 2 * sam.x() - 1;
    float u;
    if (std::abs(g_) < 0.001f)
      u = s;
    else {
      float g2 = g_ * g_;
      u = (1 + g2 - std::pow((1 - g2) / (1 + g_ * s), 2.f)) / (2 * g_);
    }

    const float cos_theta = -u;
    const float sin_theta = std::max(0.f, 1.f - cos_theta * cos_theta);
    const float phi = 2 * Constants<float>::pi() * sam.y();

    const Vector3f lwi{sin_theta * std::sin(phi), sin_theta * std::cos(phi),
                       cos_theta};

    const float phase_value = phase_func(u);

    const auto wi = CoordinateSystem(wo).Local2World(lwi);

    return BSDFSampleResult{wi, Spectrum{phase_value}, phase_value, false};
  }

  virtual float Pdf(const Vector3f& wi, const Vector3f& wo,
                    uint8_t type) const noexcept override {
    const float u = -dot(wi, wo);
    return phase_func(u);
  }

 private:
  float phase_func(float u) const {
    const float g2 = g_ * g_;
    const float dem = 1 + g2 - 2 * g_ * u;
    return (1 - g2) / (4 * Constants<float>::pi() * dem * std::sqrt(dem));
  }

  float g_ = 0.f;
  Spectrum albedo_;
};

AJ_END
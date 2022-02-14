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
#include <ajisai/core/bsdf/bsdf.h>
#include <ajisai/core/warp.h>
#include <ajisai/utility/distribution.h>
#include <ajisai/math/constants.h>
#include <numeric>

AJ_BEGIN

BSDFSampleResult BSDF::SampleBlackFringes(const Vector3f& wo, TransMode mode,
                                          const Vector3f& sam) const noexcept {
  if (dot(geometry_normal_, wo) <= 0.f) return kBSDFSampleResultInvalid;

  auto [lwi, pdf] = SquareToCosineHemisphereSample(sam.xy());

  if (pdf < std::numeric_limits<float>::epsilon())
    return kBSDFSampleResultInvalid;

  const auto wi = geometry_coord_.Local2World(lwi).normalized();
  const float factor =
      CorrectShadingNormal(geometry_normal_, shading_normal_, wi);
  const auto f = Albedo() / Constants<float>::pi() * factor;

  return BSDFSampleResult{wi, f, pdf, false};
}

Spectrum BSDF::EvalBlackFringes(const Vector3f& wi, const Vector3f& wo,
                                TransMode mode) const noexcept {
  if ((dot(geometry_normal_, wi) <= 0.f) || (dot(geometry_normal_, wo) <= 0.f))
    return {};

  return Albedo() / Constants<float>::pi();
}

float BSDF::PdfBlackFringes(const Vector3f& wi,
                            const Vector3f& wo) const noexcept {
  if ((dot(geometry_normal_, wi) > 0.f) != (dot(geometry_normal_, wo) > 0.f))
    return 0.f;

  const auto local = geometry_coord_.World2Local(wi).normalized();
  constexpr float inv_pi = 1.f / Constants<float>::pi();
  return local.z() * inv_pi;
}

Spectrum BSDF::EvalAll(const Vector3f& wi, const Vector3f& wo,
                       TransMode mode) const noexcept {
  return Eval(wi, wo, mode, kBSDFAll);
}

BSDFSampleResult BSDF::SampleAll(const Vector3f& wo, TransMode mode,
                                 const Vector3f& sam) const noexcept {
  return Sample(wo, mode, sam, kBSDFAll);
}

float BSDF::PdfAll(const Vector3f& wi, const Vector3f& wo) const noexcept {
  return Pdf(wi, wo, kBSDFAll);
}

BSDFComponent::BSDFComponent(uint8_t type) noexcept : type_(type) {}

bool BSDFComponent::Contained(uint8_t type) const noexcept {
  return (type_ & type) == type_;
}

AJ_END
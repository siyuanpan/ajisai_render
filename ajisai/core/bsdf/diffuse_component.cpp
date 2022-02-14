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
#include <ajisai/core/bsdf/diffuse_component.h>
#include <ajisai/core/warp.h>
#include <ajisai/math/constants.h>

AJ_BEGIN

DiffuseComponent::DiffuseComponent(const Spectrum& albedo) noexcept
    : BSDFComponent(BSDFComponentType::eDiffuse) {
  coef_ = albedo / Constants<float>::pi();
}

BSDFComponent::SampleResult DiffuseComponent::Sample(
    const Vector3f& lwo, TransMode mode, const Vector2f& sam) const noexcept {
  if (lwo.z() < 0) return {};

  auto [lwi, pdf] = SquareToCosineHemisphereSample(sam);

  if (pdf < 1e-3) {
    return {};
  }

  SampleResult ret;
  ret.f = coef_;
  ret.lwi = lwi;
  ret.pdf = pdf;

  return ret;
}

Spectrum DiffuseComponent::Eval(const Vector3f& lwi, const Vector3f& lwo,
                                TransMode mode) const noexcept {
  if (lwi.z() <= 0 || lwo.z() <= 0) return {};

  return coef_;
}

float DiffuseComponent::Pdf(const Vector3f& lwi,
                            const Vector3f& lwo) const noexcept {
  if (lwi.z() <= 0 || lwo.z() <= 0) return 0;

  constexpr float inv_pi = 1.f / Constants<float>::pi();
  return lwi.z() * inv_pi;
}

AJ_END
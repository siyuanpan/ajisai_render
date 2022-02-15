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
#include <ajisai/core/bsdf/plastic_bsdf.h>
#include <ajisai/core/bsdf/helper.h>
#include <ajisai/core/warp.h>
#include <ajisai/math/functions.h>
#include <mutex>

AJ_BEGIN

static std::once_flag flag;
static float kDiffuseFresnel = 0.f;

PlasticBsdf::PlasticBsdf(const Vector3f& ng, const Vector3f& ns,
                         const Spectrum& albedo, float ior, float thickness,
                         float sigma_a)
    : BSDF{ng, ns},
      albedo_(albedo),
      ior_(ior),
      thickness_(thickness),
      sigma_a_(sigma_a) {
  scaled_sigma_a_ = thickness_ * Vector3f{sigma_a_};
  auto avg =
      (scaled_sigma_a_[0] + scaled_sigma_a_[1] + scaled_sigma_a_[2]) / 3.f;
  avg_transmittance_ = std::exp(-2.0f * avg);

  std::call_once(flag, [&]() {
    kDiffuseFresnel = Fresnel::computeDiffuseFresnel(ior_, 1000000);
  });
}

BSDFSampleResult PlasticBsdf::Sample(const Vector3f& wo, TransMode mode,
                                     const Vector3f& sam,
                                     uint8_t type) const noexcept {
  if (CauseBlackFringes(wo)) {
    return SampleBlackFringes(wo, mode, sam);
  }

  const Vector3f lwo = shading_coord_.World2Local(wo).normalized();
  if (lwo.z() <= 0.f) return kBSDFSampleResultInvalid;

  bool sample_r = true;
  bool sample_t = true;

  if (!(type & eSpecular)) sample_r = false;
  if (!(type & eDiffuse)) sample_t = false;

  float eta = 1.f / ior_;
  float Fi = Fresnel::dielectricReflectance(eta, lwo.z());
  float substrate_weight = avg_transmittance_ * (1.0f - Fi);
  float specular_weight = Fi;
  float specular_probability;
  if (sample_r && sample_t)
    specular_probability =
        specular_weight / (specular_weight + substrate_weight);
  else if (sample_r)
    specular_probability = 1.0f;
  else if (sample_t)
    specular_probability = 0.0f;
  else
    return kBSDFSampleResultInvalid;

  Vector3f lwi{};
  float pdf = 0.f;
  Spectrum f{};
  bool is_delta = false;

  if (sample_r && sam.x() < specular_probability) {
    lwi = Vector3f(-lwo.x(), -lwo.y(), lwo.z());
    pdf = specular_probability;
    f = Spectrum(Fi / specular_probability);
    is_delta = true;
  } else {
    std::tie(lwi, pdf) =
        SquareToCosineHemisphereSample(Vector2f{sam.y(), sam.z()});
    float Fo = Fresnel::dielectricReflectance(eta, lwi.z());
    Spectrum diffuse_albedo = Albedo();

    f = ((1.0f - Fi) * (1.0f - Fo) * eta * eta) *
        (diffuse_albedo / (Spectrum{1.0f} - diffuse_albedo * kDiffuseFresnel));
    if (scaled_sigma_a_.max() > 0.0f)
      f *= Spectrum{Exp(scaled_sigma_a_ * (-1.0f / lwi.z() - 1.0f / lwo.z()))};

    pdf *= (1.0f - specular_probability);
    f /= 1.0f - specular_probability;
    is_delta = false;
  }

  const Vector3f wi = shading_coord_.Local2World(lwi);

  return BSDFSampleResult{wi.normalized(), f, pdf, is_delta};
}

Spectrum PlasticBsdf::Eval(const Vector3f& wi, const Vector3f& wo,
                           TransMode mode, uint8_t type) const noexcept {
  if (CauseBlackFringes(wo)) {
    return EvalBlackFringes(wi, wo, mode);
  }

  const Vector3f& lwi = shading_coord_.World2Local(wi).normalized();
  const Vector3f& lwo = shading_coord_.World2Local(wo).normalized();

  bool eval_r = true;
  bool eval_t = true;

  if (!(type & eSpecular)) eval_r = false;
  if (!(type & eDiffuse)) eval_t = false;

  float eta = 1.0f / ior_;
  float Fi = Fresnel::dielectricReflectance(eta, lwo.z());
  float Fo = Fresnel::dielectricReflectance(eta, lwi.z());

  if (eval_r && checkReflectionConstraint(lwo, lwi)) {
    return Spectrum(Fi);
  } else if (eval_t) {
    Spectrum diffuse_albedo = Albedo();

    Spectrum brdf =
        ((1.0f - Fi) * (1.0f - Fo) * eta * eta * lwi.z() *
         (1.f / Constants<float>::pi())) *
        (diffuse_albedo / (Spectrum{1.0f} - diffuse_albedo * kDiffuseFresnel));

    if (scaled_sigma_a_.max() > 0.0f)
      brdf *= Exp(scaled_sigma_a_ * (-1.0f / lwi.z() - 1.0f / lwo.z()));
    return brdf;
  } else {
    return Spectrum(0.0f);
  }
}

float PlasticBsdf::Pdf(const Vector3f& wi, const Vector3f& wo,
                       uint8_t type) const noexcept {
  if (CauseBlackFringes(wo)) {
    return PdfBlackFringes(wi, wo);
  }

  const Vector3f& lwi = shading_coord_.World2Local(wi).normalized();
  const Vector3f& lwo = shading_coord_.World2Local(wo).normalized();

  if (lwi.z() <= 0.0f || lwo.z() <= 0.0f) return 0.0f;

  bool sample_r = true;
  bool sample_t = true;

  if (!(type & eSpecular)) sample_r = false;
  if (!(type & eDiffuse)) sample_t = false;

  constexpr float inv_pi = 1.f / Constants<float>::pi();

  if (sample_r && sample_t) {
    float Fi = Fresnel::dielectricReflectance(1.0f / ior_, lwo.z());
    float substrateWeight = avg_transmittance_ * (1.0f - Fi);
    float specularWeight = Fi;
    float specularProbability =
        specularWeight / (specularWeight + substrateWeight);
    if (checkReflectionConstraint(lwo, lwi))
      return specularProbability;
    else
      return lwi.z() * inv_pi * (1.0f - specularProbability);
  } else if (sample_t) {
    return lwi.z() * inv_pi;
  } else if (sample_r) {
    return checkReflectionConstraint(lwo, lwi) ? 1.0f : 0.0f;
  } else {
    return 0.0f;
  }
}

AJ_END
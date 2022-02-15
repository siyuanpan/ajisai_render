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
#include <ajisai/core/bsdf/microfacet_reflection_component.h>
#include <ajisai/core/warp.h>
#include <ajisai/math/constants.h>

AJ_BEGIN

namespace {

inline float Sqr(float x) noexcept { return x * x; }

inline float Phi(const Vector3f& w) noexcept {
  if (w.x() <= 0.f && w.y() <= 0.f) return 0.f;

  const float ret = std::atan2(w.y(), w.x());
  return ret < 0.f ? (ret + 2 * Constants<float>::pi()) : ret;
}

inline float TanTheta(const Vector3f& w) {
  const float t = 1 - w.z() * w.z();
  if (t <= 0) return 0;
  return std::sqrt(t) / w.z();
}

class TrowbridgeReitzDistribution {
 public:
  static float D(float sin_phi_h, float cos_phi_h, float sin_theta_h,
                 float cos_theta_h, float ax, float ay) noexcept {
    const float A = Sqr(cos_phi_h / ax) + Sqr(sin_phi_h / ay);
    const float RD = Sqr(sin_theta_h) * A + Sqr(cos_theta_h);
    return 1.f / (Constants<float>::pi() * ax * ay * Sqr(RD));
  }

  static float G(float cos_phi, float sin_phi, float ax, float ay,
                 float tan_theta) noexcept {
    const float t = Sqr(ax * cos_phi) + Sqr(ay * sin_phi);
    const float sqr_val = 1 + t * Sqr(tan_theta);
    const float lambda = -0.5f + 0.5f * std::sqrt(sqr_val);
    return 1 / (1 + lambda);
  }

  static Vector3f SampleWh(const Vector3f& ve, float ax, float ay,
                           const Vector2f& sam) noexcept {
    const Vector3f vh = Vector3f{ax * ve.x(), ay * ve.y(), ve.z()}.normalized();
    const float lensq = vh.x() * vh.x() + vh.y() * vh.y();

    const Vector3f t1 = lensq > std::numeric_limits<float>::epsilon()
                            ? Vector3f(-vh.y(), vh.x(), 0) / std::sqrt(lensq)
                            : Vector3f(1, 0, 0);
    const Vector3f t2 = cross(vh, t1);

    const float r = std::sqrt(sam.x());
    const float phi = 2 * Constants<float>::pi() * sam.y();
    const float t_1 = r * std::cos(phi);
    const float _t_2 = r * std::sin(phi);
    const float s = 0.5f * (1 + vh.z());
    const float t_2 = (1 - s) * std::sqrt(1 - t_1 * t_1) + s * _t_2;

    const Vector3f nh =
        t_1 * t1 + t_2 * t2 +
        std::sqrt(std::max(0.f, 1.f - t_1 * t_1 - t_2 * t_2)) * vh;
    const Vector3f ne =
        Vector3f(ax * nh.x(), ay * nh.y(), std::max(0.f, nh.z())).normalized();

    return ne;
  }
};

template <bool eval, bool pdf>
void EvalPdf(const Vector3f& lwi, const Vector3f& lwo, const Fresnel* fresnel,
             float ax, float ay, Spectrum* eval_output,
             float* pdf_output) noexcept {
  if (lwi.z() <= 0 || lwo.z() <= 0) {
    if constexpr (eval) *eval_output = {};
    if constexpr (pdf) *pdf_output = 0.f;
    return;
  }

  const float cos_theta_i = lwi.z();
  const float cos_theta_o = lwo.z();

  const Vector3f lwh = (lwi + lwo).normalized();
  const float cos_theta_d = dot(lwi, lwh);

  const float phi_h = Phi(lwh);
  const float sin_phi_h = std::sin(phi_h);
  const float cos_phi_h = std::cos(phi_h);
  const float cos_theta_h = lwh.z();
  const float sin_theta_h =
      std::sqrt(std::max(0.f, 1 - cos_theta_h * cos_theta_h));
  const float D = TrowbridgeReitzDistribution::D(
      sin_phi_h, cos_phi_h, sin_theta_h, cos_theta_h, ax, ay);

  const float phi_i = Phi(lwi);
  const float phi_o = Phi(lwo);
  const float sin_phi_i = std::sin(phi_i), cos_phi_i = std::cos(phi_i);
  const float sin_phi_o = std::sin(phi_o), cos_phi_o = std::cos(phi_o);
  const float tan_theta_i = TanTheta(lwi);
  const float tan_theta_o = TanTheta(lwo);

  const float Go =
      TrowbridgeReitzDistribution::G(cos_phi_o, sin_phi_o, ax, ay, tan_theta_o);

  if constexpr (eval) {
    const float Gi = TrowbridgeReitzDistribution::G(cos_phi_i, sin_phi_i, ax,
                                                    ay, tan_theta_i);
    const float G = Gi * Go;

    const Spectrum F = fresnel->Eval(cos_theta_d);

    *eval_output = F * D * G / std::abs(4 * cos_theta_i * cos_theta_o);
  }

  if constexpr (pdf) {
    *pdf_output = Go * D / (4 * lwo.z());
  }
}
}  // namespace

GGXMicrofacetReflectionComponent::GGXMicrofacetReflectionComponent(
    Rc<Fresnel> fresnel, float uroughness, float vroughness) noexcept
    : BSDFComponent(BSDFComponentType::eGlossy),
      fresnel_(fresnel),
      uroughness_(uroughness),
      vroughness_(vroughness) {}

BSDFComponent::SampleResult GGXMicrofacetReflectionComponent::Sample(
    const Vector3f& lwo, TransMode mode, const Vector2f& sam) const noexcept {
  if (lwo.z() <= 0) return {};

  const Vector3f lwh =
      TrowbridgeReitzDistribution::SampleWh(lwo, uroughness_, vroughness_, sam)
          .normalized();
  if (lwh.z() <= 0) return {};

  const Vector3f lwi = (2 * dot(lwo, lwh) * lwh - lwo).normalized();
  if (lwi.z() <= 0) return {};

  SampleResult ret{};
  ret.lwi = lwi;
  EvalPdf<true, true>(lwi, lwo, fresnel_.get(), uroughness_, vroughness_,
                      &ret.f, &ret.pdf);

  return ret;
}

Spectrum GGXMicrofacetReflectionComponent::Eval(const Vector3f& lwi,
                                                const Vector3f& lwo,
                                                TransMode mode) const noexcept {
  Spectrum ret{};
  EvalPdf<true, false>(lwi, lwo, fresnel_.get(), uroughness_, vroughness_, &ret,
                       nullptr);
  return ret;
}

float GGXMicrofacetReflectionComponent::Pdf(
    const Vector3f& lwi, const Vector3f& lwo) const noexcept {
  float ret;
  EvalPdf<false, true>(lwi, lwo, fresnel_.get(), uroughness_, vroughness_,
                       nullptr, &ret);
  return ret;
}

AJ_END
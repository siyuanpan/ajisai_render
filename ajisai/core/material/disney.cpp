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
#include <ajisai/ajisai.h>
#include <ajisai/core/material/material.h>
#include <ajisai/core/bsdf/bsdf.h>
#include <ajisai/core/bsdf/aggregate_bsdf.h>
#include <ajisai/core/bsdf/microfacet_reflection_component.h>
#include <ajisai/core/intersection.h>
#include <ajisai/core/material/fresnel.h>
#include <ajisai/core/warp.h>
#include <ajisai/math/constants.h>

AJ_BEGIN

inline float CosTheta2(const Vector3f& v) { return v.z() * v.z(); }

inline float TanTheta(const Vector3f& v) {
  float temp = 1 - v.z() * v.z();
  if (temp <= 0.0f) return 0.0f;
  return std::sqrt(temp) / v.z();
}

inline float TanTheta2(const Vector3f& v) {
  float tmp = 1.f - v.z() * v.z();
  if (tmp <= 0.f) return 0.f;
  return tmp / (v.z() * v.z());
}

template <class T1, class T2>
inline auto Lerp(const T1& min, const T2& max, const float val)
    -> decltype(min + max) {
  return min * (1 - val) + max * val;
}

inline Vector3f Reflect(const Vector3f& vInci, const Vector3f& nNorm) {
  return vInci + Vector3f(2 * dot(-vInci, nNorm) * nNorm);
}

class DisneyBSDF : public BSDF {
 public:
  DisneyBSDF(const Vector3f& ng, const Vector3f& ns, const Spectrum& base_color,
             float subsurface, float metallic, float specular,
             float specular_tint, float roughness, float anisotropic,
             float sheen, float sheen_tint, float clearcoat,
             float clearcoat_gloss) noexcept
      : BSDF{ng, ns},
        base_color_{base_color},
        subsurface_{subsurface},
        metallic_{metallic},
        specular_{specular},
        specular_tint_{specular_tint},
        roughness_{roughness},
        anisotropic_{anisotropic},
        sheen_{sheen},
        sheen_tint_{sheen_tint},
        clearcoat_{clearcoat},
        clearcoat_gloss_{clearcoat_gloss} {}
  virtual Spectrum Albedo() const override { return base_color_; }

  virtual bool HasDiffuseComponent() const override { return metallic_ < 1.f; }

  virtual BSDFSampleResult Sample(const Vector3f& wo, TransMode mode,
                                  const Vector3f& sam,
                                  uint8_t type) const noexcept override {
    // if (!(type & BSDFComponentType::eSpecular)) {
    // return kBSDFSampleResultInvalid;
    // }

    const Vector3f lwo = shading_coord_.World2Local(wo).normalized();
    Vector3f lwi, lwh;
    bool sample_coat = false;

    Vector3f remapped_sam = sam;

    if (clearcoat_ > 0.f) {
      float coat_weight = clearcoat_ / (clearcoat_ + Albedo().Luminance());
      float fresnel_coat = FresnelSchlickCoat(std::abs(lwo.z()));
      float prob_coat = (fresnel_coat * coat_weight) /
                        (fresnel_coat * coat_weight +
                         (1.f - fresnel_coat) * (1.f - coat_weight));

      if (sam[1] < prob_coat) {
        sample_coat = true;
        remapped_sam[1] /= prob_coat;

        float coat_rough = std::lerp(0.005f, 0.1f, clearcoat_gloss_);
        float coat_pdf;
        lwh = GGXSampleVisibleNormal(lwo, remapped_sam[0], remapped_sam[1],
                                     &coat_pdf, coat_rough);
        lwi = Reflect(-lwo, lwh);
      } else {
        sample_coat = false;
        remapped_sam[1] = (sam[1] - prob_coat) / (1.f - prob_coat);
      }
    }

    if (!sample_coat) {
      float microfacet_pdf;
      float roughness = std::clamp(roughness_, 0.02f, 1.0f);

      lwh = GGXSampleVisibleNormal(lwo, remapped_sam[0], remapped_sam[1],
                                   &microfacet_pdf, roughness * roughness);

      float normalRef = std::lerp(0.0f, 0.08f, specular_);
      float ODotH = dot(lwo, lwh);
      float probSpec = FresnelSchlick(ODotH, normalRef);

      if (remapped_sam[2] <= probSpec) {
        lwi = Reflect(-lwo, lwh);
      } else {
        lwi = squareToCosineHemisphere(
            Vector2f{remapped_sam[0], remapped_sam[1]});

        if (lwo.z() < 0.0f) lwi.z() *= -1.0f;
      }
    }

    if (lwi.z() <= 0.0f) {
      return kBSDFSampleResultInvalid;
    }

    const auto wi = shading_coord_.Local2World(lwi);
    const auto f = EvalAll(wi, wo, mode);
    const float pdf = PdfAll(wi, wo);

    return BSDFSampleResult{wi, f, pdf, false};
  }

  virtual Spectrum Eval(const Vector3f& wi, const Vector3f& wo, TransMode mode,
                        uint8_t type) const noexcept override {
    const auto lwi = shading_coord_.World2Local(wi).normalized();
    const auto lwo = shading_coord_.World2Local(wo).normalized();
    if (std::abs(lwi.z()) < std::numeric_limits<float>::epsilon() ||
        std::abs(lwo.z()) < std::numeric_limits<float>::epsilon())
      return {};

    float roughness = std::clamp(roughness_, 0.02f, 1.f);
    Spectrum untinted_spec_albedo{Albedo().Luminance()};
    Spectrum spec_albedo =
        Lerp(Lerp(Albedo(), untinted_spec_albedo, 1.0f - specular_tint_),
             Albedo(), metallic_);
    Spectrum sheen_albedo = Lerp(untinted_spec_albedo, Albedo(), sheen_tint_);

    Vector3f wh = (lwo + lwi).normalized();
    float o_dot_h = dot(lwo, wh);
    float i_dot_h = dot(lwi, wh);
    float one_minus_o_dot_h = 1.f - o_dot_h;
    Spectrum grazing_spec_albedo =
        Lerp(spec_albedo, untinted_spec_albedo, metallic_);
    spec_albedo =
        Lerp(spec_albedo, untinted_spec_albedo,
             one_minus_o_dot_h * one_minus_o_dot_h * one_minus_o_dot_h);

    // Sheen term
    float F = FresnelSchlick(o_dot_h, 0.0f);
    Spectrum sheen_term = F * sheen_ * sheen_albedo;

    return (1.0f - metallic_) *
               (Albedo() * Lerp(DiffuseTerm(lwi, lwo, i_dot_h, roughness),
                                SubsurfaceTerm(lwi, lwo, i_dot_h, roughness),
                                subsurface_) +
                sheen_term) +
           spec_albedo * SpecularTerm(lwi, lwo, wh, o_dot_h, roughness) +
           Spectrum(ClearCoatTerm(lwi, lwo, wh, i_dot_h, clearcoat_gloss_));
  }

  virtual float Pdf(const Vector3f& wi, const Vector3f& wo,
                    uint8_t type) const noexcept override {
    const auto lwi = shading_coord_.World2Local(wi).normalized();
    const auto lwo = shading_coord_.World2Local(wo).normalized();
    Vector3f wh = (lwo + lwi).normalized();
    if (wh.max() <= 0.f) return 0.f;

    float roughness = std::clamp(roughness_, 0.02f, 1.f);

    float microfacet_pdf = GGXPdfVisibleNormal(lwo, wh, roughness * roughness);
    float pdf = 0.f;
    float dwhdwi = 1.f / (4.f * std::abs(dot(lwi, wh)));
    float spec_pdf = microfacet_pdf * dwhdwi;

    float normal_ref = std::lerp(0.f, 0.08f, specular_);
    float o_dot_h = dot(lwo, wh);
    float prob_spec = FresnelSchlick(o_dot_h, normal_ref);

    constexpr float inv_pi = 1.f / Constants<float>::pi();
    pdf += spec_pdf * prob_spec;
    pdf += std::abs(lwi.z()) * inv_pi * (1.f - prob_spec);

    if (clearcoat_ > 0.f) {
      float coat_weight = clearcoat_ / (clearcoat_ + Albedo().Luminance());
      float fresnel_coat = FresnelSchlickCoat(std::abs(wo.z()));
      float prob_coat =
          (fresnel_coat * coat_weight) /
          (fresnel_coat * coat_weight + (1 - fresnel_coat) * (1 - coat_weight));
      float coat_rough = std::lerp(0.005f, 0.10f, clearcoat_gloss_);
      float coat_half_pdf = GGXPdfVisibleNormal(wo, wh, coat_rough);
      float coat_pdf = coat_half_pdf * dwhdwi;

      pdf *= 1.0f - prob_coat;
      pdf += coat_pdf * prob_coat;
    }

    return pdf;
  }

 protected:
  float DiffuseTerm(const Vector3f& wi, const Vector3f& wo, const float i_dot_h,
                    const float roughness) const {
    if (metallic_ == 1.f) return 0.f;

    float one_minus_cosl = 1.f - std::abs(wi.z());
    float one_minus_cosl_sqr = one_minus_cosl * one_minus_cosl;
    float one_minus_cosv = 1.f - std::abs(wo.z());
    float one_minus_cosv_sqr = one_minus_cosv * one_minus_cosv;
    float F_D90 = 0.5f + 2.f * i_dot_h * roughness;

    constexpr float inv_pi = 1.f / Constants<float>::pi();

    return inv_pi *
           (1.f + (F_D90 - 1.f) * one_minus_cosl_sqr * one_minus_cosl_sqr *
                      one_minus_cosl) *
           (1.f + (F_D90 - 1.f) * one_minus_cosv_sqr * one_minus_cosv_sqr *
                      one_minus_cosv);
  }

  float SpecularTerm(const Vector3f& wi, const Vector3f& wo, const Vector3f& wh,
                     const float o_dot_h, const float roughness) const {
    if (wi.z() * wo.z() <= 0.f) return 0.f;

    float D = GGX_D(wh, roughness * roughness);
    if (D == 0.f) return 0.f;

    float normal_ref = std::lerp(0.f, 0.08f, specular_);
    float F = FresnelSchlick(o_dot_h, normal_ref);

    float rough_for_G = (0.5f + 0.5f * roughness);
    float G = GGX_G(wi, wo, wh, rough_for_G * rough_for_G);

    return F * D * G / (4.f * std::abs(wi.z()) * std::abs(wo.z()));
  }

  float SubsurfaceTerm(const Vector3f& wi, const Vector3f& wo,
                       const float i_dot_h, const float roughness) const {
    if (subsurface_ == 0.0f) return 0.0f;

    float one_minus_cosl = 1.0f - std::abs(wi.z());
    float one_minus_cosl_sqr = one_minus_cosl * one_minus_cosl;
    float one_minus_cosv = 1.0f - std::abs(wo.z());
    float one_minus_cosv_sqr = one_minus_cosv * one_minus_cosv;
    float F_ss90 = i_dot_h * i_dot_h * roughness;

    float S = (1.0f + (F_ss90 - 1.0f) * one_minus_cosl_sqr *
                          one_minus_cosl_sqr * one_minus_cosl) *
              (1.0f + (F_ss90 - 1.0f) * one_minus_cosv_sqr *
                          one_minus_cosv_sqr * one_minus_cosv);

    constexpr float inv_pi = 1.f / Constants<float>::pi();

    return inv_pi * 1.25f *
           (S * (1.0f / (std::abs(wo.z()) + std::abs(wi.z())) - 0.5f) + 0.5f);
  }

  float ClearCoatTerm(const Vector3f& wi, const Vector3f& wo,
                      const Vector3f& wh, const float i_dot_h,
                      const float roughness) const {
    if (clearcoat_ == 0.0f) return 0.0f;

    float rough = Lerp(0.005f, 0.1f, roughness);

    float D = GGX_D(wh, rough);
    if (D == 0.0f) return 0.0f;

    float one_minus_cosD = 1.0f - i_dot_h;
    float one_minus_cosD_sqr = one_minus_cosD * one_minus_cosD;
    float F = FresnelSchlickCoat(i_dot_h);
    float G = GGX_G(wo, wi, wh, 0.25f);

    return clearcoat_ * D * F * G /
           (4.0f * std::abs(wi.z()) * std::abs(wo.z()));
  }

  float GGX_D(const Vector3f& wh, float alpha) const {
    if (wh.z() <= 0.f) return 0.f;

    const auto tan_theta2 = TanTheta2(wh);
    const auto cos_theta2 = CosTheta2(wh);

    const float root = alpha / (cos_theta2 * (alpha * alpha + tan_theta2));

    constexpr float inv_pi = 1.f / Constants<float>::pi();
    return inv_pi * (root * root);
  }

  float GGXPdfVisibleNormal(const Vector3f& wi, const Vector3f& h,
                            float alpha) const {
    float D = GGX_D(h, alpha);

    return SmithG(wi, h, alpha) * std::abs(dot(wi, h)) * D /
           (std::abs(wi.z()) + 1e-4f);
  }

  Vector3f GGXSampleVisibleNormal(const Vector3f& _wi, float u1, float u2,
                                  float* pPdf, float Alpha) const {
    // Stretch wi
    Vector3 wi =
        Vector3f(Alpha * _wi.x(), Alpha * _wi.y(), _wi.z()).normalized();

    // Get polar coordinates
    float Theta = 0, Phi = 0;
    if (wi.z() < float(0.99999f)) {
      Theta = std::acos(wi.z());
      Phi = std::atan2(wi.y(), wi.x());
    }
    float SinPhi = std::sin(Phi), CosPhi = std::cos(Phi);

    // Simulate P22_{wi}(Slope.x, Slope.y, 1, 1)
    Vector2f Slope = ImportanceSampleGGXVisibleNormalUnit(Theta, u1, u2);

    // Step 3: rotate
    Slope = Vector2f(CosPhi * Slope.x() - SinPhi * Slope.y(),
                     SinPhi * Slope.x() + CosPhi * Slope.y());

    // Unstretch
    Slope.x() *= Alpha;
    Slope.y() *= Alpha;

    // Compute normal
    float Normalization =
        (float)1.f /
        std::sqrt(Slope.x() * Slope.x() + Slope.y() * Slope.y() + (float)1.0);

    Vector3f H = Vector3f(-Slope.x() * Normalization,
                          -Slope.y() * Normalization, Normalization);

    *pPdf = GGXPdfVisibleNormal(_wi, H, Alpha);

    return H;
  }

  Vector2f ImportanceSampleGGXVisibleNormalUnit(float thetaI, float u1,
                                                float u2) const {
    Vector2f Slope;

    // Special case (normal incidence)
    if (thetaI < 1e-4f) {
      float SinPhi, CosPhi;
      float R = std::sqrt(std::max(u1 / ((1 - u1) + 1e-6f), 0.0f));
      SinPhi = std::sin(2 * Constants<float>::pi() * u2);
      CosPhi = std::cos(2 * Constants<float>::pi() * u2);
      return Vector2f(R * CosPhi, R * SinPhi);
    }

    // Precomputations
    float TanThetaI = std::tan(thetaI);
    float a = 1 / TanThetaI;
    float G1 = 2.0f / (1.0f + std::sqrt(std::max(1.0f + 1.0f / (a * a), 0.0f)));

    // Simulate X component
    float A = 2.0f * u1 / G1 - 1.0f;
    if (std::abs(A) == 1) A -= (A >= 0.0f ? 1.0f : -1.0f) * 1e-4f;

    float Temp = 1.0f / (A * A - 1.0f);
    float B = TanThetaI;
    float D =
        std::sqrt(std::max(B * B * Temp * Temp - (A * A - B * B) * Temp, 0.0f));
    float Slope_x_1 = B * Temp - D;
    float Slope_x_2 = B * Temp + D;
    Slope.x() =
        (A < 0.0f || Slope_x_2 > 1.0f / TanThetaI) ? Slope_x_1 : Slope_x_2;

    // Simulate Y component
    float S;
    if (u2 > 0.5f) {
      S = 1.0f;
      u2 = 2.0f * (u2 - 0.5f);
    } else {
      S = -1.0f;
      u2 = 2.0f * (0.5f - u2);
    }

    // Improved fit
    float z = (u2 * (u2 * (u2 * (-0.365728915865723) + 0.790235037209296) -
                     0.424965825137544) +
               0.000152998850436920) /
              (u2 * (u2 * (u2 * (u2 * 0.169507819808272 - 0.397203533833404) -
                           0.232500544458471) +
                     1) -
               0.539825872510702);

    Slope.y() = S * z * std::sqrt(1.0f + Slope.x() * Slope.x());

    return Slope;
  }

  float FresnelSchlick(const float cos_D, const float normalReflectance) const {
    float one_minus_cosD = 1.f - cos_D;
    float one_minus_cosDSqr = one_minus_cosD * one_minus_cosD;
    float fresnel = normalReflectance + (1.f - normalReflectance) *
                                            one_minus_cosDSqr *
                                            one_minus_cosDSqr * one_minus_cosD;
    float fresnel_conductor = FresnelConductor(cos_D, 0.4f, 1.6f);

    return std::lerp(fresnel, fresnel_conductor, metallic_);
  }

  float FresnelSchlickCoat(const float cosD) const {
    float one_minus_cosD = 1.0f - cosD;
    float one_minus_cosD_sqr = one_minus_cosD * one_minus_cosD;
    float fresnel = 0.04f + (1.0f - 0.04f) * one_minus_cosD_sqr *
                                one_minus_cosD_sqr * one_minus_cosD;

    return fresnel;
  }

  float FresnelConductor(float cosi, const float& eta, const float k) const {
    float tmp = (eta * eta + k * k) * cosi * cosi;
    float Rparl2 =
        (tmp - (2.f * eta * cosi) + 1) / (tmp + (2.f * eta * cosi) + 1);
    float tmp_f = eta * eta + k * k;
    float Rperp2 = (tmp_f - (2.f * eta * cosi) + cosi * cosi) /
                   (tmp_f + (2.f * eta * cosi) + cosi * cosi);

    return 0.5f * (Rparl2 + Rperp2);
  }

  float SmithG(const Vector3f& v, const Vector3f& wh, float alpha) const {
    const float tan_theta = std::abs(TanTheta(v));

    if (tan_theta == 0.0f) return 1.0f;

    if (dot(v, wh) * v.z() <= 0) return 0.0f;

    const float root = alpha * tan_theta;
    return 2.0f / (1.0f + std::sqrt(1.0f + root * root));
  }

  float GGX_G(const Vector3f& wi, const Vector3f& wo, const Vector3f& wh,
              float alpha) const {
    return SmithG(wo, wh, alpha) * SmithG(wi, wh, alpha);
  }

 private:
  Spectrum base_color_;
  float subsurface_;
  float metallic_;
  float specular_;
  float specular_tint_;
  float roughness_;
  float anisotropic_;
  float sheen_;
  float sheen_tint_;
  float clearcoat_;
  float clearcoat_gloss_;
};

class Disney : public Material {
 public:
  explicit Disney(Rc<const Texture2D> base_color,
                  Rc<const Texture2D> subsurface, Rc<const Texture2D> metallic,
                  Rc<const Texture2D> specular,
                  Rc<const Texture2D> specular_tint,
                  Rc<const Texture2D> roughness,
                  Rc<const Texture2D> anisotropic, Rc<const Texture2D> sheen,
                  Rc<const Texture2D> sheen_tint, Rc<const Texture2D> clearcoat,
                  Rc<const Texture2D> clearcoat_gloss)
      : base_color_{base_color},
        subsurface_{subsurface},
        metallic_{metallic},
        specular_{specular},
        specular_tint_{specular_tint},
        roughness_{roughness},
        anisotropic_{anisotropic},
        sheen_{sheen},
        sheen_tint_{sheen_tint},
        clearcoat_{clearcoat},
        clearcoat_gloss_{clearcoat_gloss} {}

  virtual ShadingPoint Shade(const PrimitiveIntersection& inct,
                             MemoryArena& arena) const override {
    const auto base_color = base_color_->SampleSpectrum(inct.uv);
    const auto subsurface = subsurface_->SampleReal(inct.uv);
    const auto metallic = metallic_->SampleReal(inct.uv);
    const auto specular = specular_->SampleReal(inct.uv);
    const auto specular_tint = specular_tint_->SampleReal(inct.uv);
    const auto roughness = roughness_->SampleReal(inct.uv);
    const auto anisotropic = anisotropic_->SampleReal(inct.uv);
    const auto sheen = sheen_->SampleReal(inct.uv);
    const auto sheen_tint = sheen_tint_->SampleReal(inct.uv);
    const auto clearcoat = clearcoat_->SampleReal(inct.uv);
    const auto clearcoat_gloss = clearcoat_gloss_->SampleReal(inct.uv);

    BSDF* bsdf = arena.Create<DisneyBSDF>(
        inct.geometry_normal, inct.shading_normal, base_color, subsurface,
        metallic, specular, specular_tint, roughness, anisotropic, sheen,
        sheen_tint, clearcoat, clearcoat_gloss);
    // new DisneyBSDF(inct.geometry_normal, inct.shading_normal,
    //                             base_color, subsurface, metallic, specular,
    //                             specular_tint, roughness, anisotropic, sheen,
    //                             sheen_tint, clearcoat, clearcoat_gloss);

    ShadingPoint sp{};
    sp.bsdf = bsdf;
    sp.shading_normal = inct.shading_normal;

    return std::move(sp);
  }

 private:
  Rc<const Texture2D> base_color_;
  Rc<const Texture2D> subsurface_;
  Rc<const Texture2D> metallic_;
  Rc<const Texture2D> specular_;
  Rc<const Texture2D> specular_tint_;
  Rc<const Texture2D> roughness_;
  Rc<const Texture2D> anisotropic_;
  Rc<const Texture2D> sheen_;
  Rc<const Texture2D> sheen_tint_;
  Rc<const Texture2D> clearcoat_;
  Rc<const Texture2D> clearcoat_gloss_;
};

Rc<Material> CreateDisney(
    Rc<const Texture2D> base_color, Rc<const Texture2D> subsurface,
    Rc<const Texture2D> metallic, Rc<const Texture2D> specular,
    Rc<const Texture2D> specular_tint, Rc<const Texture2D> roughness,
    Rc<const Texture2D> anisotropic, Rc<const Texture2D> sheen,
    Rc<const Texture2D> sheen_tint, Rc<const Texture2D> clearcoat,
    Rc<const Texture2D> clearcoat_gloss) {
  return RcNew<Disney>(
      std::move(base_color), std::move(subsurface), std::move(metallic),
      std::move(specular), std::move(specular_tint), std::move(roughness),
      std::move(anisotropic), std::move(sheen), std::move(sheen_tint),
      std::move(clearcoat), std::move(clearcoat_gloss));
}

AJ_END
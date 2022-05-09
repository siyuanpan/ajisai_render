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
#include <ajisai/core/intersection.h>
#include <ajisai/core/material/fresnel.h>
#include <ajisai/math/functions.h>
#include <ajisai/core/bsdf/bsdf.h>

AJ_BEGIN

class GlassBSDF : public BSDF {
 public:
  GlassBSDF(const Vector3f& ng, const Vector3f& ns,
            Rc<DielectricFresnel> fresnel, const Spectrum& color_reflection,
            const Spectrum& color_refraction) noexcept
      : BSDF{ng, ns},
        fresnel_{fresnel},
        color_reflection_{color_reflection},
        color_refraction_{color_refraction} {}

  virtual Spectrum Albedo() const override {
    return 0.5f * (color_reflection_ + color_refraction_);
  }

  virtual BSDFSampleResult Sample(const Vector3f& wo, TransMode mode,
                                  const Vector3f& sam,
                                  uint8_t type) const noexcept override {
    const Vector3f& lwo = shading_coord_.World2Local(wo).normalized();
    const Vector3f& normal =
        lwo.z() > 0 ? Vector3f{0.f, 0.f, 1.f} : Vector3f{0.f, 0.f, -1.f};

    const auto fr = fresnel_->Eval(lwo.z());

    if (sam.x() < fr[0]) {
      const auto lwi = Vector3f{-lwo.x(), -lwo.y(), lwo.z()};
      const auto wi = shading_coord_.Local2World(lwi);
      const auto f = color_reflection_ * fr / std::abs(lwi.z());

      const float factor =
          CorrectShadingNormal(geometry_normal_, shading_normal_, wi);

      return BSDFSampleResult{wi, f * factor, fr[0], true};
    }

    const float eta_i = lwo.z() > 0 ? fresnel_->eta_o() : fresnel_->eta_i();
    const float eta_t = lwo.z() > 0 ? fresnel_->eta_i() : fresnel_->eta_o();
    const float eta = eta_i / eta_t;

    // refraction dir
    std::optional<Vector3f> opt_wi;
    {
      const auto cos_theta_i = std::abs(lwo.z());
      const auto sin_theta_i_2 = std::max(0.f, 1.f - cos_theta_i * cos_theta_i);
      const auto sin_theta_t_2 = eta * eta * sin_theta_i_2;
      if (sin_theta_t_2 > 1.f) {
        opt_wi = std::nullopt;
      } else {
        const auto cos_theta_t = std::sqrt(1 - sin_theta_t_2);
        opt_wi = (eta * cos_theta_i - cos_theta_t) * normal - eta * lwo;
      }
    }

    if (!opt_wi) {
      return kBSDFSampleResultInvalid;
    }

    const auto lwi = opt_wi->normalized();

    const auto dir = shading_coord_.Local2World(lwi);
    const auto f =
        eta * eta * color_refraction_ * (1.f - fr[0]) / std::abs(lwi.z());
    const auto pdf = 1.f - fr[0];

    const float factor =
        CorrectShadingNormal(geometry_normal_, shading_normal_, dir);

    return BSDFSampleResult{dir, f * factor, pdf, true};
  }

  virtual Spectrum Eval(const Vector3f& wi, const Vector3f& wo, TransMode mode,
                        uint8_t type) const noexcept override {
    return Spectrum{};
  }

  virtual float Pdf(const Vector3f& wi, const Vector3f& wo,
                    uint8_t type) const noexcept override {
    return 0.f;
  }

 private:
  Rc<DielectricFresnel> fresnel_;
  Spectrum color_reflection_;
  Spectrum color_refraction_;
};

class Glass : public Material {
 public:
  explicit Glass(Rc<const Texture2D> color_reflection,
                 Rc<const Texture2D> color_refraction, Rc<const Texture2D> ior)
      : color_reflection_(color_reflection),
        color_refraction_(color_refraction),
        ior_(ior) {}

  virtual ShadingPoint Shade(const PrimitiveIntersection& inct) const override {
    const auto ior = ior_->SampleReal(inct.uv);

    const auto color_reflection = color_reflection_->SampleSpectrum(inct.uv);
    Spectrum color_refraction;
    if (color_reflection_ == color_refraction_) {
      color_refraction = aj::sqrt(color_reflection);
    } else {
      color_refraction = color_refraction_->SampleSpectrum(inct.uv);
      color_refraction = aj::sqrt(color_refraction);
    }

    const Rc<DielectricFresnel> fresnel = RcNew<DielectricFresnel>(ior, 1.f);

    BSDF* bsdf = new GlassBSDF(inct.geometry_normal, inct.shading_normal,
                               fresnel, color_reflection, color_refraction);

    ShadingPoint sp{};
    sp.bsdf = bsdf;
    sp.shading_normal = inct.shading_normal;

    return std::move(sp);
  }

 private:
  Rc<const Texture2D> color_reflection_;
  Rc<const Texture2D> color_refraction_;
  Rc<const Texture2D> ior_;
};

Rc<Material> CreateGlass(Rc<const Texture2D> color_reflection,
                         Rc<const Texture2D> color_refraction,
                         Rc<const Texture2D> ior) {
  return RcNew<Glass>(std::move(color_reflection), std::move(color_refraction),
                      std::move(ior));
}

AJ_END
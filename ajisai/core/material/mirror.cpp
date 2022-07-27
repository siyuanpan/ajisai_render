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

AJ_BEGIN

class MirrorBSDF : public BSDF {
 public:
  MirrorBSDF(const Vector3f& ng, const Vector3f& ns,
             const Spectrum& kr) noexcept
      : BSDF{ng, ns}, kr_{kr} {}
  virtual Spectrum Albedo() const override { return kr_; }

  virtual bool HasDiffuseComponent() const override { return false; }

  virtual BSDFSampleResult Sample(const Vector3f& wo, TransMode mode,
                                  const Vector3f& sam,
                                  uint8_t type) const noexcept override {
    if (!(type & BSDFComponentType::eSpecular)) {
      return kBSDFSampleResultInvalid;
    }

    const Vector3f& lwo = shading_coord_.World2Local(wo).normalized();
    if (lwo.z() <= 0.f) {
      return kBSDFSampleResultInvalid;
    }

    const auto lwi = Vector3f{-lwo.x(), -lwo.y(), lwo.z()};
    const auto wi = shading_coord_.Local2World(lwi);

    const float factor =
        CorrectShadingNormal(geometry_normal_, shading_normal_, wi);

    return BSDFSampleResult{wi, kr_ * factor / std::abs(lwi.z()), 1.f, true};
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
  Spectrum kr_;
};

class Mirror : public Material {
 public:
  explicit Mirror(Rc<const Texture2D> kr) : kr_(kr) {}

  virtual ShadingPoint Shade(const PrimitiveIntersection& inct,
                             MemoryArena& arena) const override {
    const auto kr = kr_->SampleSpectrum(inct.uv);

    BSDF* bsdf =
        arena.Create<MirrorBSDF>(inct.geometry_normal, inct.shading_normal, kr);
    // new MirrorBSDF(inct.geometry_normal, inct.shading_normal, kr);

    ShadingPoint sp{};
    sp.bsdf = bsdf;
    sp.shading_normal = inct.shading_normal;

    return std::move(sp);
  }

 private:
  Rc<const Texture2D> kr_;
};

Rc<Material> CreateMirror(Rc<const Texture2D> kr) {
  return RcNew<Mirror>(std::move(kr));
}

AJ_END
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

class Metal : public Material {
 public:
  explicit Metal(Rc<const Texture2D> color, Rc<const Texture2D> k,
                 Rc<const Texture2D> eta, float uroughness, float vroughness)
      : color_(color),
        k_(k),
        eta_(eta),
        uroughness_(uroughness),
        vroughness_(vroughness) {}

  virtual ShadingPoint Shade(const PrimitiveIntersection& inct) const override {
    const auto color = color_->SampleSpectrum(inct.uv);
    const auto k = k_->SampleSpectrum(inct.uv);
    const auto eta = eta_->SampleSpectrum(inct.uv);

    // const Rc<Fresnel> fresnel = RcNew<ConductorFresnel>(Spectrum{1.f}, eta,
    // k);
    const Rc<Fresnel> fresnel =
        RcNew<ColoredConductorFresnel>(color, Spectrum{1.f}, eta, k);

    AggregateBSDF* bsdf = new AggregateBSDF(inct.geometry_normal,
                                            inct.shading_normal, Spectrum{1.f});
    bsdf->AddComponent(1.f, RcNew<GGXMicrofacetReflectionComponent>(
                                fresnel, uroughness_, vroughness_));

    ShadingPoint sp{};
    sp.bsdf = bsdf;
    sp.shading_normal = inct.shading_normal;

    return std::move(sp);
  }

 private:
  Rc<const Texture2D> color_;
  Rc<const Texture2D> k_;
  Rc<const Texture2D> eta_;
  float uroughness_, vroughness_;
};

Rc<Material> CreateMetal(Rc<const Texture2D> color, Rc<const Texture2D> k,
                         Rc<const Texture2D> eta, float uroughness,
                         float vroughness) {
  return RcNew<Metal>(std::move(color), std::move(k), std::move(eta),
                      uroughness, vroughness);
}

AJ_END
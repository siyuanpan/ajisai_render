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
#include <ajisai/core/bsdf/plastic_bsdf.h>
#include <ajisai/core/intersection.h>
#include <ajisai/core/bsdf/diffuse_component.h>

AJ_BEGIN

class Plastic : public Material {
 public:
  explicit Plastic(Rc<const Texture2D>&& albedo, float ior, float thickness,
                   float sigma_a)
      : albedo_(std::move(albedo)),
        ior_(ior),
        thickness_(thickness),
        sigma_a_(sigma_a) {}

  virtual ShadingPoint Shade(const PrimitiveIntersection& inct,
                             MemoryArena& arena) const override {
    const auto albedo = albedo_->SampleSpectrum(inct.uv);
    auto bsdf =
        arena.Create<PlasticBsdf>(inct.geometry_normal, inct.shading_normal,
                                  albedo, ior_, thickness_, sigma_a_);
    // new PlasticBsdf(inct.geometry_normal, inct.shading_normal,
    //                             albedo, ior_, thickness_, sigma_a_);

    ShadingPoint sp{};
    sp.bsdf = bsdf;
    sp.shading_normal = inct.shading_normal;

    return std::move(sp);
  }

 private:
  Rc<const Texture2D> albedo_;
  float ior_, thickness_, sigma_a_;
};

Rc<Material> CreatePlastic(Rc<const Texture2D>&& albedo, float ior,
                           float thickness, float sigma_a) {
  return RcNew<Plastic>(std::move(albedo), ior, thickness, sigma_a);
}

AJ_END
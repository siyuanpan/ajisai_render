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
#include <ajisai/core/intersection.h>
#include <ajisai/core/bsdf/diffuse_component.h>

AJ_BEGIN

class Diffuse : public Material {
 public:
  explicit Diffuse(Rc<const Texture2D> albedo) : albedo_(albedo) {}

  virtual ShadingPoint Shade(const PrimitiveIntersection& inct) const override {
    const auto albedo = albedo_->SampleSpectrum(inct.uv);
    auto bsdf = new BSDF(inct.geometry_normal, inct.shading_normal, albedo);
    bsdf->AddComponent(1.f, RcNew<DiffuseComponent>(albedo));

    ShadingPoint sp{};
    sp.bsdf = bsdf;
    sp.shading_normal = inct.shading_normal;

    return std::move(sp);
  }

 private:
  Rc<const Texture2D> albedo_;
};

Rc<Material> CreateDiffuse(Rc<const Texture2D> albedo) {
  return RcNew<Diffuse>(albedo);
}

AJ_END
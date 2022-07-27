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
#include <ajisai/core/intersection.h>
#include <ajisai/core/bsdf/diffuse_component.h>

AJ_BEGIN

class NullBsdf : public BSDF {
 public:
  NullBsdf(const Vector3f& ng, const Vector3f& ns) noexcept : BSDF{ng, ns} {}

  virtual Spectrum Albedo() const override { return Spectrum{}; }

  virtual bool HasDiffuseComponent() const override { return false; }

  virtual BSDFSampleResult Sample(const Vector3f& wo, TransMode mode,
                                  const Vector3f& sam,
                                  uint8_t type) const noexcept override {
    return BSDFSampleResult{-wo, Spectrum{1.f}, 1.f, true};
  }

  virtual Spectrum Eval(const Vector3f& wi, const Vector3f& wo, TransMode mode,
                        uint8_t type) const noexcept override {
    return Spectrum{};
  }

  virtual float Pdf(const Vector3f& wi, const Vector3f& wo,
                    uint8_t type) const noexcept override {
    return 0.f;
  }
};

class Null : public Material {
 public:
  explicit Null() {}

  virtual ShadingPoint Shade(const PrimitiveIntersection& inct,
                             MemoryArena& arena) const override {
    BSDF* bsdf =
        arena.Create<NullBsdf>(inct.geometry_normal, inct.shading_normal);
    // new NullBsdf(inct.geometry_normal, inct.shading_normal);

    ShadingPoint sp{};
    sp.bsdf = bsdf;
    sp.shading_normal = inct.shading_normal;

    return std::move(sp);
  }
};

Rc<Material> CreateNull() { return RcNew<Null>(); }

AJ_END
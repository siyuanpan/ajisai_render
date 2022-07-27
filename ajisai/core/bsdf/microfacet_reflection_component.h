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
#pragma once
#include <ajisai/ajisai.h>
#include <ajisai/core/bsdf/bsdf.h>
#include <ajisai/core/material/fresnel.h>
#include <ajisai/math/spectrum.h>

AJ_BEGIN

class GGXMicrofacetReflectionComponent : public BSDFComponent {
 public:
  GGXMicrofacetReflectionComponent(Rc<Fresnel> fresnel, float uroughness,
                                   float vroughness) noexcept;

  virtual bool HasDiffuseComponent() const override { return false; }

  virtual BSDFComponent::SampleResult Sample(
      const Vector3f& lwo, TransMode mode,
      const Vector2f& sam) const noexcept override;

  virtual Spectrum Eval(const Vector3f& lwi, const Vector3f& lwo,
                        TransMode mode) const noexcept override;

  virtual float Pdf(const Vector3f& lwi,
                    const Vector3f& lwo) const noexcept override;

 private:
  Rc<Fresnel> fresnel_;
  float uroughness_, vroughness_;
};

AJ_END

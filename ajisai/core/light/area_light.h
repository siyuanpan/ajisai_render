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
#include <ajisai/core/light/light.h>
#include <ajisai/core/geometry/geometry.h>
#include <ajisai/math/spectrum.h>

AJ_BEGIN

class AreaLight : public Light {
 public:
  AreaLight(const Geometry *geometry, Spectrum radiance, int32_t power);

  virtual Spectrum Radiance(const Vector3f &pos, const Vector3f &nor,
                            const Vector2f &uv,
                            const Vector3f &light_to_out) const noexcept;

  virtual const AreaLight *AsArea() const noexcept override { return this; }

  virtual LightSampleResult Sample(const Vector3f &ref,
                                   Sampler *sampler) const noexcept override;

  virtual LightEmitResult SampleEmit(Sampler *sampler) const noexcept override;

  virtual float Pdf(const Vector3f &ref, const Vector3f &pos,
                    const Vector3f &normal) const noexcept;

  virtual bool IsFinite() const noexcept override { return true; }

  virtual bool IsDelta() const noexcept override { return false; }

  void PdfBdpt(const Vector3f &direction, const Vector3f &pos,
               const Vector3f &normal, float *direct_pdf_a = nullptr,
               float *emission_pdf_w = nullptr) const noexcept;

 private:
  const Geometry *geometry_;
  Spectrum radiance_;
  int32_t power_;
};

AJ_END
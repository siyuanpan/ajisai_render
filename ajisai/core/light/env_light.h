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
#include <ajisai/utility/distribution.h>
#include <ajisai/math/spectrum.h>

#include <optional>

AJ_BEGIN

class EnvLight : public Light {
 public:
  EnvLight(Rc<const Texture2D> &&texture, float rot, float);

  virtual const EnvLight *AsEnv() const noexcept override { return this; }

  virtual Spectrum Radiance(const Vector3f &ref,
                            const Vector3f &ref2light) const noexcept;

  virtual LightSampleResult Sample(const Vector3f &ref,
                                   Sampler *sampler) const noexcept override;

  virtual LightEmitResult SampleEmit(Sampler *sampler) const noexcept override;

  virtual float Pdf(const Vector3f &ref,
                    const Vector3f &ref2light) const noexcept;

  virtual void Process(const Bounds3f &bounds) noexcept override {
    aabb_ = bounds;
  }

 private:
  Rc<const Texture2D> texture_;
  float env_map_rot_ = 0.f;
  std::optional<Distribution2D> sampler_;
  Bounds3f aabb_;
  float scaler_ = 1.f;
};

AJ_END
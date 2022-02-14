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
#include <ajisai/math/vector3.h>
#include <ajisai/math/angle.h>
#include <ajisai/math/spectrum.h>

AJ_BEGIN

class AreaLight;
class EnvLight;
class Sampler;
class Texture2D;

struct LightSampleResult {
  Vector3f ref;
  Vector3f pos;
  Vector3f normal;
  Vector2f uv;
  Spectrum radiance;
  float pdf;

  bool Valid() const noexcept { return pdf != 0.f; }

  Vector3f Ref2Light() const noexcept { return (pos - ref).normalized(); }
};

inline const LightSampleResult kLightSampleResultNull = {
    Vector3f{}, Vector3f{}, Vector3f{}, Vector2f{}, Spectrum{}, 0};

class Light {
 public:
  virtual ~Light() = default;

  virtual const AreaLight* AsArea() const noexcept { return nullptr; }

  virtual const EnvLight* AsEnv() const noexcept { return nullptr; }

  virtual LightSampleResult Sample(const Vector3f& ref,
                                   Sampler* sampler) const noexcept = 0;

  virtual void Process(const Bounds3f&) noexcept {}
};

AJISAI_API Rc<Light> CreateDirectionalLight(const Vector3f& dir,
                                            const Rad<float>& angle,
                                            Spectrum radiance, int32_t power);

AJISAI_API Rc<Light> CreateEnvLight(Rc<const Texture2D>&& texture, float rot);

AJ_END
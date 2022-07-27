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
#include <ajisai/core/light/area_light.h>
#include <ajisai/core/sampler/sampler.h>
#include <ajisai/core/warp.h>

AJ_BEGIN

AreaLight::AreaLight(const Geometry *geometry, Spectrum radiance, int32_t power)
    : geometry_(geometry), radiance_(radiance), power_(power) {}

Spectrum AreaLight::Radiance(const Vector3f &pos, const Vector3f &nor,
                             const Vector2f &uv,
                             const Vector3f &light_to_out) const noexcept {
  return dot(nor, light_to_out) > 0 ? radiance_ : Spectrum{0.f};
}

LightSampleResult AreaLight::Sample(const Vector3f &ref,
                                    Sampler *sampler) const noexcept {
  float pdf_area = 0.f;
  auto inct = geometry_->Sample(ref, &pdf_area, sampler->Next3D());

  if (dot(inct.geometry_normal, ref - inct.pos) <= 0) {
    return kLightSampleResultNull;
  }

  const Vector3f inct2ref = ref - inct.pos;
  const float dist2 = inct2ref.dot();

  const float pdf =
      pdf_area * dist2 / std::abs(dot(inct.geometry_normal, inct2ref));

  return LightSampleResult{ref,     inct.pos,  inct.geometry_normal,
                           inct.uv, radiance_, pdf};
}

LightEmitResult AreaLight::SampleEmit(Sampler *sampler) const noexcept {
  float pdf_pos = 0.f;
  auto inct = geometry_->Sample(&pdf_pos, sampler->Next3D());

  auto [lwi, pdf_dir] = SquareToCosineHemisphereSample(sampler->Next2D());

  const Vector3f dir = CoordinateSystem(inct.geometry_normal).Local2World(lwi);

  return LightEmitResult{
      inct.pos, dir, inct.geometry_normal, inct.uv, radiance_, pdf_pos, pdf_dir,
  };
}

float AreaLight::Pdf(const Vector3f &ref, const Vector3f &pos,
                     const Vector3f &normal) const noexcept {
  if (dot(ref - pos, normal) <= 0.f) return 0.f;

  const float area_pdf = geometry_->Pdf(ref, pos);

  const Vector3f light2ref = ref - pos;
  const float dist2 = light2ref.dot();
  const float abscos = std::abs(dot(light2ref, normal));
  const float area_to_solid_angle_factor = dist2 / abscos;

  return area_pdf * area_to_solid_angle_factor;
}

AJ_END
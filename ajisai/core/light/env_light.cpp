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
#include <ajisai/core/light/env_light.h>
#include <ajisai/core/texture2d/texture2d.h>
#include <ajisai/core/sampler/sampler.h>
#include <ajisai/core/warp.h>

AJ_BEGIN

EnvLight::EnvLight(Rc<const Texture2D> &&texture, float rot, float scaler)
    : texture_(std::move(texture)), env_map_rot_(rot), scaler_(scaler) {
  auto width = texture_->Width();
  auto height = texture_->Height();

  auto weights = BoxNew<float[]>(width * height);

  for (size_t y = 0; y < height; ++y) {
    float v = (y + 0.5f) / float(height);
    float sin_theta =
        std::sin(Constants<float>::pi() * (y + 0.5f) / float(height));
    for (size_t x = 0; x < width; ++x) {
      float u = (x + 0.5f) / float(width);
      weights[x + width * y] =
          texture_->SampleSpectrum(Vector2f{u, v}).Luminance() * sin_theta;
    }
  }

  sampler_.emplace(weights.get(), width, height);
}

Spectrum EnvLight::Radiance(const Vector3f &ref,
                            const Vector3f &ref2light) const noexcept {
  constexpr float inv_2_pi = 1.f / (2.f * Constants<float>::pi());
  constexpr float inv_pi = 1.f / Constants<float>::pi();

  const Vector3f dir = ref2light.normalized();
  const float theta = std::acos(std::clamp(dir.y(), -1.f, 1.f));
  Vector2f uv{
      (Constants<float>::pi() + std::atan2(dir.z(), dir.x())) * inv_2_pi,
      theta * inv_pi};

  uv += Vector2f{env_map_rot_ / 180 * Constants<float>::pi() * inv_2_pi, 0.f};
  if (uv.x() > 1.f) uv.x() -= 1.f;
  if (uv.x() < 0.f) uv.x() += 1.f;

  return scaler_ * texture_->SampleSpectrum(uv);
}

LightSampleResult EnvLight::Sample(const Vector3f &ref,
                                   Sampler *sampler) const noexcept {
  float u, v, pdf;
  sampler_->SampleContinuous(sampler->Next1D(), sampler->Next1D(), &u, &v,
                             &pdf);

  if (pdf == 0.f) {
    return kLightSampleResultNull;
  }

  constexpr float inv_2_pi = 1.f / (2.f * Constants<float>::pi());
  constexpr float two_pi = 2.f * Constants<float>::pi();
  u -= env_map_rot_ / 180 * Constants<float>::pi() * inv_2_pi;
  if (u > 1.f) u -= 1.f;
  if (u < 0.f) u += 1.f;
  float phi = u * two_pi;
  float theta = v * Constants<float>::pi();
  float sin_theta = std::sin(theta);

  pdf = sin_theta != 0.f ? pdf / (2.f * Constants<float>::pi() *
                                  Constants<float>::pi() * sin_theta)
                         : 0.f;

  const Vector3f dir{-std::sin(theta) * std::cos(phi), std::cos(theta),
                     -std::sin(theta) * std::sin(phi)};

  auto radius = (aabb_.max() - aabb_.center()).length();

  return LightSampleResult{ref,
                           ref + 2.f * radius * dir,
                           -dir,
                           {},
                           texture_->SampleSpectrum(Vector2f{u, v}),
                           pdf};
}

LightEmitResult EnvLight::SampleEmit(Sampler *sampler) const noexcept {
  float u, v, pdf;
  sampler_->SampleContinuous(sampler->Next1D(), sampler->Next1D(), &u, &v,
                             &pdf);

  if (pdf == 0.f) {
    return {Vector3f{}, Vector3f{}, Vector3f{}, Vector2f{}, Spectrum{}, 0, 0};
  }

  constexpr float inv_2_pi = 1.f / (2.f * Constants<float>::pi());
  constexpr float two_pi = 2.f * Constants<float>::pi();
  u -= env_map_rot_ / 180 * Constants<float>::pi() * inv_2_pi;
  if (u > 1.f) u -= 1.f;
  if (u < 0.f) u += 1.f;
  float phi = u * two_pi;
  float theta = v * Constants<float>::pi();
  float sin_theta = std::sin(theta);

  pdf = sin_theta != 0.f ? pdf / (2.f * Constants<float>::pi() *
                                  Constants<float>::pi() * sin_theta)
                         : 0.f;

  Vector3f dir{-std::sin(theta) * std::cos(phi), std::cos(theta),
               -std::sin(theta) * std::sin(phi)};

  dir = -dir;
  auto radius = (aabb_.max() - aabb_.center()).length();

  const Vector2f disk_sam = squareToUniformDiskConcentric(sampler->Next2D());

  auto dir_coord = CoordinateSystem(-dir);

  const Vector3f pos = aabb_.center() + (disk_sam.x() * dir_coord.x() +
                                         disk_sam.y() * dir_coord.y() - dir) *
                                            radius;

  return LightEmitResult{
      pos,
      dir,
      dir.normalized(),
      {},
      texture_->SampleSpectrum(Vector2f{u, v}),
      1.f / (Constants<float>::pi() * radius * radius),
      pdf,
  };
}

float EnvLight::Pdf(const Vector3f &ref,
                    const Vector3f &ref2light) const noexcept {
  constexpr float inv_2_pi = 1.f / (2.f * Constants<float>::pi());
  constexpr float inv_pi = 1.f / Constants<float>::pi();

  const Vector3f dir = ref2light.normalized();
  float theta = std::acos(std::clamp(dir.y(), -1.f, 1.f));
  Vector2f uv{
      (Constants<float>::pi() + std::atan2(dir.z(), dir.x())) * inv_2_pi,
      theta * inv_pi};
  uv += Vector2f{env_map_rot_ / 180 * Constants<float>::pi() * inv_2_pi, 0.f};
  if (uv.x() > 1.f) uv.x() -= 1.f;
  if (uv.x() < 0.f) uv.x() += 1.f;

  float sin_theta = std::sin(theta);

  return sampler_->Pdf(uv[0], uv[1]) /
         (2.f * Constants<float>::pi() * Constants<float>::pi() * sin_theta);
}

Rc<Light> CreateEnvLight(Rc<const Texture2D> &&texture, float rot,
                         float scaler) {
  return RcNew<EnvLight>(std::move(texture), rot, scaler);
}

AJ_END
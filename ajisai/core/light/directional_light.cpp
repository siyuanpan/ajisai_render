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
#include <ajisai/core/light/light.h>
#include <ajisai/core/coordinate_system.h>
#include <ajisai/core/sampler/sampler.h>
#include <ajisai/core/warp.h>
#include <ajisai/math/vector3.h>
#include <ajisai/math/bounds.h>
#include <ajisai/math/angle.h>

AJ_BEGIN

class DirectionalLight : public Light {
 public:
  explicit DirectionalLight(const Vector3f& dir, const Rad<float>& angle,
                            Spectrum radiance, int32_t power)
      : dir_{dir.normalized()},
        frame_{dir_},
        angle_cos_{std::cos((float)angle)},
        radiance_{radiance},
        power_{power} {}

  virtual LightSampleResult Sample(const Vector3f& ref,
                                   Sampler* sampler) const noexcept override {
    const Vector3f ref2light =
        frame_.Local2World(UniformSampleCone(sampler->Next2D(), angle_cos_));

    const float pdf = UniformConePDF(angle_cos_);
    auto radius = (aabb_.max() - aabb_.center()).length();

    return LightSampleResult{
        ref, ref + 2.f * radius * ref2light, {}, {}, radiance_, pdf};
  }

  virtual LightEmitResult SampleEmit(Sampler* sampler) const noexcept override {
    const Vector3f ref2light =
        frame_.Local2World(UniformSampleCone(sampler->Next2D(), angle_cos_));

    const float pdf_dir = UniformConePDF(angle_cos_);

    auto radius = (aabb_.max() - aabb_.center()).length();
    const Vector2f disk_sam = squareToUniformDiskConcentric(sampler->Next2D());
    auto dir_coord = CoordinateSystem(ref2light);
    const Vector3f pos =
        aabb_.center() + (disk_sam.x() * dir_coord.x() +
                          disk_sam.y() * dir_coord.y() + ref2light) *
                             radius;

    return LightEmitResult{
        pos,     -ref2light, -ref2light.normalized(),
        {},      radiance_,  1.f / (Constants<float>::pi() * radius * radius),
        pdf_dir,
    };
  }

  virtual void Process(const Bounds3f& bounds) noexcept override {
    aabb_ = bounds;
  }

 private:
  Vector3f dir_;
  const CoordinateSystem frame_;
  float angle_cos_;
  Spectrum radiance_;
  int32_t power_;
  Bounds3f aabb_;
};

Rc<Light> CreateDirectionalLight(const Vector3f& dir, const Rad<float>& angle,
                                 Spectrum radiance, int32_t power) {
  return RcNew<DirectionalLight>(dir, angle, radiance, power);
}

AJ_END
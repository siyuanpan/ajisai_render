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
#include <ajisai/core/coordinate_system.h>
#include <ajisai/math/vector3.h>
#include <ajisai/math/spectrum.h>

#include <vector>

AJ_BEGIN
class BSDFComponent;

inline float CorrectShadingNormal(const Vector3f& ng, const Vector3f& ns,
                                  const Vector3f& wi) noexcept {
  const float denom = std::abs(dot(ng, wi));
  return denom == 0 ? 1.f : std::abs(dot(ns, wi) / denom);
}

struct BSDFSampleResult {
  explicit BSDFSampleResult() noexcept {}

  BSDFSampleResult(const Vector3f& dir, const Spectrum& f, float pdf,
                   bool is_delta) noexcept
      : dir(dir), f(f), pdf(pdf), is_delta(is_delta) {}

  Vector3f dir;           // scattering direction
  Spectrum f;             // bsdf value
  float pdf = 0;          // pdf w.r.t. solid angle
  bool is_delta = false;  // is f/pdf delta function?

  bool Invalid() const noexcept { return dir.max() <= 0; }
};

inline const BSDFSampleResult kBSDFSampleResultInvalid =
    BSDFSampleResult{Vector3f{}, Spectrum{}, 0, false};

class BSDF {
 public:
  BSDF(const Vector3f& ng, const Vector3f& ns, const Spectrum& albedo)
      : geometry_normal_(ng),
        geometry_coord_{ng},
        shading_normal_{ns},
        shading_coord_{ns},
        albedo_{albedo} {}

  void AddComponent(float weight, Rc<BSDFComponent> component);

  Spectrum Albedo() const;

  BSDFSampleResult SampleAll(const Vector3f& wo, TransMode mode,
                             const Vector3f& sam) const noexcept;

 protected:
  bool CauseBlackFringes(const Vector3f& w) const noexcept {
    return (dot(geometry_normal_, w) > 0) != (dot(shading_normal_, w) > 0);
  }

 private:
  Vector3f geometry_normal_, shading_normal_;
  const CoordinateSystem geometry_coord_, shading_coord_;
  Spectrum albedo_;
  std::vector<Rc<BSDFComponent>> components_;
  std::vector<float> weights_;
};

class BSDFComponent {
 public:
  struct SampleResult {
    Vector3f lwi;
    Spectrum f;
    float pdf = 0;

    bool Valid() const noexcept { return pdf != 0; }
  };

  virtual ~BSDFComponent() = default;

  virtual SampleResult Sample(const Vector3f& lwo, TransMode mode,
                              const Vector2f& sam) const noexcept = 0;

  virtual Spectrum Eval(const Vector3f& lwi, const Vector3f& lwo,
                        TransMode mode) const noexcept = 0;

  virtual float Pdf(const Vector3f& lwi,
                    const Vector3f& lwo) const noexcept = 0;
};

AJ_END
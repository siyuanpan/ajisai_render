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

#ifndef AJISAI_CORE_CAMERA_H_
#define AJISAI_CORE_CAMERA_H_

#include <Ajisai/Core/Film.h>
#include <Ajisai/Core/Geometry.h>
#include <Ajisai/Math/Math.h>

#include "Ajisai/Core/Random.h"
#include "Ajisai/Core/Warp.h"

namespace Ajisai::Core {

// struct Intersection {
//   float t = std::numeric_limits<float>::infinity();
//   int meshId = -1;
//   int triId = -1;
//   Math::Vector3f Ng;
//   Math::Vector2f uv;
//   Math::Vector3f p;
// };

class Camera {
 public:
  explicit Camera(const Math::Vector3f& ori, const Math::Vector3f& tar,
                  float focus_distance, const Math::Vector3f& upDir, float fov,
                  /*float aspectRatio*/ const Math::Vector2f& res,
                  float lensRadius = 0.f)
      : origin{ori},
        resolution(res),
        lensRadius{lensRadius},
        focus_distance{focus_distance} {
    film = std::make_shared<Film>(Math::Vector2i{int(res.x()), int(res.y())});

    look = (ori - tar).normalized();
    right = Math::cross(upDir, look).normalized();
    up = Math::cross(look, right).normalized();

    const float halfH = std::tan(fov * 0.5f) * focus_distance;
    const float halfW = res.aspectRatio() * halfH;
    lowerLeftCorner =
        origin - halfW * right - halfH * up - focus_distance * look;
    horitonal = 2.f * halfW * right;
    vertical = 2.f * halfH * up;
  }

  Ray GenerateRay(float s, float t) const {
    PCG32 rnd(s * t);
    auto rd = lensRadius * squareToUniformDiskConcentric(
                               {rnd.next_float(), rnd.next_float()});
    auto offset = right * rd.x() + up * rd.y();
    return Ray{origin + offset, lowerLeftCorner + s * horitonal + t * vertical -
                                    offset - origin};
  }

  std::shared_ptr<Film> GetFilm() const { return film; }

  float A() const { return horitonal.length() * vertical.length(); }

  void Pdf_We(const Ray& ray, float* pdfPos, float* pdfDir) const {
    float cosTheta = Math::dot(-look, ray.d);
    auto pFocus = ray.Point((lensRadius == 0 ? 1 : focus_distance) / cosTheta);

    if (cosTheta <= 0) {
      return;
    }

    float d = 1 / cosTheta;
    float lensArea = lensRadius == 0 ? 1.0f
                                     : lensRadius * lensRadius *
                                           Math::Constants<float>::pi();
    *pdfPos = 1 / lensArea;
    *pdfDir = 1 / A() * d * d / cosTheta;
  }

  Math::Spectrum We(const Ray& ray, Math::Vector2f* pRaster) const {
    float cosTheta = Math::dot(-look, ray.d);
    Math::Vector3f pFocus =
        ray.Point((lensRadius == 0 ? 1 : focus_distance) / cosTheta);
    if (cosTheta <= 0) {
      return Math::Spectrum(0);
    }

    Math::Vector2f raster{Math::dot(pFocus - origin, right),
                          Math::dot(pFocus - origin, up)};
    raster = (raster + Math::Vector2f{1.f}) / 2.f;

    auto bounds = film->Dimension();
    raster = Math::Vector2f{float(bounds[0]), float(bounds[1])} * raster;

    if (raster.x() < 0.f || raster.x() > bounds.x() || raster.y() < 0.f ||
        raster.y() > bounds.y())
      return {};

    *pRaster = raster;
    float lensArea = lensRadius == 0 ? 1.0f
                                     : lensRadius * lensRadius *
                                           Math::Constants<float>::pi();
    return Math::Spectrum(1 / (A() * lensArea * std::pow(cosTheta, 4)));
  }

  void Sample_Wi(const Math::Vector2f& u, const Intersect& ref,
                 CameraSamplingRecord* sample, VisibilityTester* tester) const {
    Math::Vector2f pLens = lensRadius * squareToUniformDiskConcentric(u);

    Math::Vector3f pLensWorld = origin + pLens.x() * right + pLens.y() * up;
    sample->normal = -look;
    sample->wi = pLensWorld - ref.p;
    float dist = sample->wi.length();
    sample->wi /= dist;
    float lensArea = lensRadius == 0 ? 1.0f
                                     : lensRadius * lensRadius *
                                           Math::Constants<float>::pi();
    tester->shadowRay = Ray(pLensWorld, -sample->wi, Ray::Eps(),
                            dist * (1.0 - Ray::ShadowEps()));
    sample->pdf =
        (dist * dist) / (lensArea * abs(dot(sample->normal, sample->wi)));
    sample->I = We(tester->shadowRay, &sample->pos);
  }

 private:
  Math::Vector3f origin;
  Math::Vector3f right, up, look;
  Math::Vector3f lowerLeftCorner;
  Math::Vector3f horitonal, vertical;
  Math::Vector2f resolution;
  float lensRadius;
  float focus_distance;

  std::shared_ptr<Film> film;
};
}  // namespace Ajisai::Core

#endif
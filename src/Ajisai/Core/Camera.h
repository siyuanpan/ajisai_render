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
      : origin{ori}, resolution(res), lensRadius{lensRadius} {
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

 private:
  Math::Vector3f origin;
  Math::Vector3f right, up, look;
  Math::Vector3f lowerLeftCorner;
  Math::Vector3f horitonal, vertical;
  Math::Vector2f resolution;
  float lensRadius;

  std::shared_ptr<Film> film;
};
}  // namespace Ajisai::Core

#endif
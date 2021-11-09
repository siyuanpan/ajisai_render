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

#include "Ajisai/Core/Random.h"
#include "Ajisai/Core/Warp.h"
#include "Ajisai/Math/Math.h"

namespace Ajisai::Core {

struct Ray {
  Math::Vector3f o, d;
  float t_min, t_max;

  Ray() = default;
  Ray(const Math::Vector3f& o, const Math::Vector3f& d,
      float t_min = 0.001 /*ray bias*/,
      float t_max = std::numeric_limits<float>::infinity())
      : o(o), d(d.normalized()), t_min(t_min), t_max(t_max) {}

  auto Point(float t) const { return o + t * d; }
};

struct Intersection {
  float t = std::numeric_limits<float>::infinity();
  int meshId = -1;
  int triId = -1;
  Math::Vector3f Ng;
  Math::Vector2f uv;

  //   void computeScatteringFunctions(const Math::Vector3f& wo, const
  //   Math::Vector3f& p, const Triangle& triangle, const Intersection&
  //   intersection) {

  //   }
};

class Camera {
 public:
  explicit Camera(const Math::Vector3f& ori, const Math::Vector3f& tar,
                  float focus_distance, const Math::Vector3f& upDir, float fov,
                  float aspectRatio, float lensRadius = 0.f)
      : origin{ori}, lensRadius{lensRadius} {
    look = (ori - tar).normalized();
    right = Math::cross(upDir, look).normalized();
    up = Math::cross(look, right).normalized();

    // const float focusDis = (eye - view).length();
    // std::cout << "dis " << focus_distance << std::endl;
    // std::cout << "std::tan(fov * 0.5f) " << std::tan(fov * 0.5f) <<
    // std::endl; std::cout << "aspectRatio : " << aspectRatio << std::endl;

    const float halfH = std::tan(fov * 0.5f) * focus_distance;
    const float halfW = aspectRatio * halfH;
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

 private:
  Math::Vector3f origin;
  Math::Vector3f right, up, look;
  Math::Vector3f lowerLeftCorner;
  Math::Vector3f horitonal, vertical;
  float lensRadius;
};
}  // namespace Ajisai::Core

#endif
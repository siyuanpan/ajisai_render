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

#ifndef AJISAI_CORE_GEOMETRY_H_
#define AJISAI_CORE_GEOMETRY_H_

// #include <Ajisai/Core/BSDF.h>
// #include <Ajisai/Core/Mesh.h>
#include <Ajisai/Math/Math.h>
// #include <memory>

namespace Ajisai::Core {

class Mesh;

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
  Math::Vector3f p;
  const Mesh* mesh;

  Math::Spectrum Le(const Math::Vector3f& wo) const;
};

// struct SurfaceInteraction {
//   Math::Vector3f wo;
//   Math::Vector3f p;
//   Math::Vector2f texCoord;
//   Math::Vector3f Ng;
//   Math::Vector3f Ns;
//   std::shared_ptr<BSDF> bsdf;
//   float rayBias = 1e-5f;
//   SurfaceInteraction(const Math::Vector3f& wo, const Math::Vector3f& p,
//                      const Triangle& triangle, const Intersection&
//                      intersection)
//       : wo(wo), p(p) {
//     texCoord = triangle.lerpTexCoord(intersection.uv);
//     Ns = triangle.lerpNormal(intersection.uv);
//     Ng = triangle.Ng;
//   }
//   Ray SpawnRay(const Math::Vector3f& w) const {
//     return Ray{p, w, rayBias / std::abs(Math::dot(w, Ng))};
//   }
// };

struct LightSamplingRecord {
  Math::Spectrum Li;
  Math::Vector3f wi;
  float pdf;
  Math::Vector3f normal;
  Ray shadowRay;
};
}  // namespace Ajisai::Core

#endif

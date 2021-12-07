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

#include <Ajisai/Core/BSDF.h>
// #include <Ajisai/Core/Mesh.h>
#include <Ajisai/Math/Math.h>
#include <Ajisai/Util/Ptr.h>

#include <cstring>

namespace Ajisai::Core {

class Mesh;
class BSDF;
class Scene;

struct Ray {
  Math::Vector3f o, d;
  float t_min, t_max;

  Ray() = default;
  Ray(const Math::Vector3f& o, const Math::Vector3f& d,
      float t_min = 0.001 /*ray bias*/,
      float t_max = std::numeric_limits<float>::infinity())
      : o(o), d(d.normalized()), t_min(t_min), t_max(t_max) {}

  auto Point(float t) const { return o + t * d; }

  static inline float Eps() { return 1e-4f; }
  static inline float ShadowEps() { return 1e-4f; }
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

struct Triangle {
  std::array<Math::Vector3f, 3> v;
  std::array<Math::Vector2f, 3> texCoords;
  std::array<Math::Vector3f, 3> Ns;
  Math::Vector3f Ng;
  Math::Vector3f lerpNormal(const Math::Vector2f& uv) const {
    return (1 - uv.x() - uv.y()) * Ns[0] + uv.x() * Ns[1] + uv.y() * Ns[2];
  }
  Math::Vector2f lerpTexCoord(const Math::Vector2f& uv) const {
    return (1 - uv.x() - uv.y()) * texCoords[0] + uv.x() * texCoords[1] +
           uv.y() * texCoords[2];
  }
  Math::Vector3f lerpVert(const Math::Vector2f& uv) const {
    return (1 - uv.x() - uv.y()) * v[0] + uv.x() * v[1] + uv.y() * v[2];
  }
  float area() const {
    auto e1 = v[1] - v[0];
    auto e2 = v[2] - v[0];
    return Math::cross(e1, e2).length() * 0.5f;
  }

  Math::Bounds3f Bounds() const {
    Math::Bounds3f bounds(Math::Vector3f{std::numeric_limits<float>::max()},
                          Math::Vector3f{std::numeric_limits<float>::lowest()});
    for (std::size_t i = 0; i != 3; ++i) {
      bounds.min() = Math::min(bounds.min(), v[i]);
      bounds.max() = Math::max(bounds.max(), v[i]);
    }

    return bounds;
  }

  auto Centroid() -> Math::Vector3f const { return Bounds().center(); }

  bool Intersect(const Ray& ray, Intersection* intersection) const {
    bool hit = false;
    auto v1 = v[0];
    auto v2 = v[1];
    auto v3 = v[2];
    auto e1 = v2 - v1;
    auto e2 = v3 - v1;
    auto Ng = Math::cross(e1, e2).normalized();
    float a, f, u, v;
    auto h = Math::cross(ray.d.normalized(), e2);
    a = Math::dot(e1, h);
    if (a > -1e-6f && a < 1e-6f) return false;
    f = 1.0f / a;
    auto s = ray.o - v1;
    u = f * Math::dot(s, h);
    if (u < 0.0 || u > 1.0) return false;
    auto q = Math::cross(s, e1);
    v = f * Math::dot(ray.d.normalized(), q);
    if (v < 0.0 || u + v > 1.0) return false;
    float t = f * Math::dot(e2, q);
    if (t > ray.t_min && t < ray.t_max) {
      if (t < intersection->t) {
        intersection->Ng = Ng;
        intersection->t = t;
        intersection->uv = Math::Vector2f{u, v};
        hit = true;
      }
    }
    return hit;
  }
};

struct Intersect {
  Math::Vector3f wo;
  Math::Vector3f p;
  Math::Vector3f Ng;
  // Math::Vector2f uv;
  Intersect() = default;
  Intersect(const Math::Vector3f& p) : p(p) {}
  Intersect(const Math::Vector3f& wo, const Math::Vector3f& p) : wo(wo), p(p) {}

  // [[nodiscard]] Ray SpawnRay(const vec3& w, Float rayBias = Eps()) const {
  //   return Ray(p, w, rayBias / abs(dot(w, Ng)), Inf);
  // }
  // [[nodiscard]] Ray SpawnTo(const vec3& q, Float rayBias = Eps()) const {
  //   auto w = (q - p);
  //   return Ray(p, w, rayBias / abs(dot(w, Ng)) / length(w), 1 - ShadowEps());
  // }
  Ray SpawnRay(const Math::Vector3f& w) const {
    return Ray{p, w, Ray::Eps() / std::abs(Math::dot(w, Ng))};
  }
};

struct SurfaceInteraction : Intersect {
  using Intersect::Intersect;
  using Intersect::Ng;
  // Math::Vector3f wo;
  // Math::Vector3f p;
  Math::Vector2f texCoord;
  // Math::Vector3f Ng;
  Math::Vector3f Ns;
  // std::shared_ptr<BSDF> bsdf;
  Util::Ptr<BSDF> bsdf;
  const Mesh* mesh;
  float rayBias = 1e-5f;
  SurfaceInteraction(const Math::Vector3f& wo, const Math::Vector3f& p,
                     const Triangle& triangle, const Intersection& intersection)
      : /*wo(wo), p(p)*/ Intersect{wo, p}, mesh(intersection.mesh) {
    texCoord = triangle.lerpTexCoord(intersection.uv);
    Ns = triangle.lerpNormal(intersection.uv);
    Ng = triangle.Ng;
  }
  SurfaceInteraction(const SurfaceInteraction& si) {
    memcpy(this, &si, sizeof(SurfaceInteraction));
  }
  SurfaceInteraction& operator=(const SurfaceInteraction& si) {
    memcpy(this, &si, sizeof(SurfaceInteraction));
    return *this;
  }

  Math::Spectrum Le(const Math::Vector3f& wo) const;
  // Ray SpawnRay(const Math::Vector3f& w) const {
  //   return Ray{p, w, rayBias / std::abs(Math::dot(w, Ng))};
  // }
};

struct LightSamplingRecord {
  Math::Spectrum Li;
  Math::Vector3f wi;
  float pdf;
  Math::Vector3f normal;
  Ray shadowRay;
};

struct CameraSamplingRecord {
  Math::Vector3f wi;
  Math::Spectrum I;
  Math::Vector3f normal;
  Math::Vector2f pos;  // the uv coordinate of the sampled position or the
                       // raster position (0,0) to (bound.x, boubd.y)
  float pdf;
};

struct VisibilityTester {
  Ray shadowRay{};
  VisibilityTester() = default;
  VisibilityTester(const Intersect& p1, const Intersect& p2) {
    auto w = p2.p - p1.p;
    auto dist = w.length();
    w /= dist;
    shadowRay = Ray(p1.p, w, Ray::Eps() / abs(dot(w, p1.Ng)),
                    dist * (1.0 - Ray::ShadowEps()));
  }
  [[nodiscard]] bool visible(const Scene& scene) const;
  [[nodiscard]] Math::Spectrum Tr(const Scene& scene) const;
};

}  // namespace Ajisai::Core

#endif

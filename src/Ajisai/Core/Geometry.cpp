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

#include <Ajisai/Core/Geometry.h>
#include <Ajisai/Core/Mesh.h>
#include <Ajisai/Core/Scene.h>

namespace Ajisai::Core {

Math::Spectrum Intersection::Le(const Math::Vector3f& wo) const {
  return mesh->Le(wo);
}

Math::Spectrum SurfaceInteraction::Le(const Math::Vector3f& wo) const {
  return mesh->Le(wo);
}

[[nodiscard]] bool VisibilityTester::visible(const Scene& scene) const {
  return !scene.Occlude(shadowRay);
}
[[nodiscard]] Math::Spectrum VisibilityTester::Tr(const Scene& scene) const {
  return scene.Occlude(shadowRay) ? Math::Spectrum(0) : Math::Spectrum(1);
}

Math::Spectrum DifferentialGeom::emit(const Math::Vector3f& dir) const {
  return _areaLight ? _areaLight->Emit(dir) : Math::Spectrum{0.f};
}

bool Triangle::Intersect(const Ray& ray, DifferentialGeom* diffGeom) const {
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
    if (t < diffGeom->_dist) {
      diffGeom->_geomNormal = Ng;
      diffGeom->_dist = t;
      diffGeom->_uv = Math::Vector2f{u, v};
      hit = true;
    }
  }
  return hit;
}

}  // namespace Ajisai::Core
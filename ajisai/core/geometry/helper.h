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
#include <ajisai/core/intersection.h>
#include <ajisai/core/ray.h>
#include <ajisai/math/vector3.h>

AJ_BEGIN

inline bool OccludeTriangle(const Ray& ray, const Vector3f& v0,
                            const Vector3f& v1, const Vector3f& v2) {
  auto e0 = v1 - v0;
  auto e1 = v2 - v0;
  auto n = cross(e0, e1).normalized();
  float a, f, u, v;
  auto h = cross(ray.d, e1);
  a = dot(e0, h);
  if (a > -1e-6f && a < 1e-6f) return false;
  f = 1.f / a;
  auto s = ray.o - v0;
  u = f * dot(s, h);
  if (u < 0.f || u > 1.f) return false;
  auto q = cross(s, e0);
  v = f * dot(ray.d, q);
  if (v < 0.f || u + v > 1.f) return false;
  float t = f * dot(e1, q);
  if (t > ray.t_min && t < ray.t_max) {
    return true;
  }

  return false;
}

inline bool IntersectWithTriangle(const Ray& ray, const Vector3f& v0,
                                  const Vector3f& v1, const Vector3f& v2,
                                  GeometryIntersection* inct) {
  bool hit = false;
  //   auto v0 = A;
  //   auto v1 = B;
  //   auto v2 = C;
  auto e0 = v1 - v0;
  auto e1 = v2 - v0;
  auto n = cross(e0, e1).normalized();
  float a, f, u, v;
  auto h = cross(ray.d, e1);
  a = dot(e0, h);
  if (a > -1e-6f && a < 1e-6f) return false;
  f = 1.f / a;
  auto s = ray.o - v0;
  u = f * dot(s, h);
  if (u < 0.f || u > 1.f) return false;
  auto q = cross(s, e0);
  v = f * dot(ray.d, q);
  if (v < 0.f || u + v > 1.f) return false;
  float t = f * dot(e1, q);
  //   printf(
  //       "u : %f, v : %f, t : %f  ray.t_min : %f, ray.t_max : %f, inct->t :
  //       %f\n", u, v, t, ray.t_min, ray.t_max, inct->t);
  if (t > ray.t_min && t < ray.t_max) {
    if (t < inct->t) {
      inct->t = t;
      inct->geometry_normal = n;
      inct->uv = Vector2f{u, v};
      hit = true;
    }
  }

  return hit;
}

AJ_END
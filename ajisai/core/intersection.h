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
#include <ajisai/core/bsdf/bsdf.h>
#include <ajisai/core/ray.h>
#include <ajisai/math/vector3.h>

AJ_BEGIN

class Primitive;
class Material;
class Medium;

struct Intersection {
  Vector3f pos;
  Vector3f geometry_normal;
  Vector3f shading_normal;
  Vector2f uv;

  Intersection() = default;

  Vector3f EpsOffset(const Vector3f& dir) const noexcept {
    if (dot(dir, geometry_normal) > 0.f) {
      return pos + geometry_normal * Ray::Eps();
    }
    return pos - geometry_normal * Ray::Eps();
  }
};

struct GeometryIntersection : Intersection {
  float t = std::numeric_limits<float>::infinity();
  Vector3f wr;
};

struct PrimitiveIntersection : GeometryIntersection {
  const Primitive* primitive = nullptr;
  const Material* material = nullptr;
  const Medium* medium_in = nullptr;
  const Medium* medium_out = nullptr;

  const Medium* WrMedium() const noexcept {
    return dot(wr, geometry_normal) >= 0 ? medium_out : medium_in;
  }

  const Medium* GetMedium(const Vector3f& d) const noexcept {
    return dot(d, geometry_normal) >= 0 ? medium_out : medium_in;
  }
};

struct DifferentialGeom {};

struct ShadingPoint {
  const BSDF* bsdf = nullptr;

  Vector3f shading_normal;

  // TODO: option bssrdf

  ShadingPoint() = default;

  ShadingPoint(ShadingPoint&& other) {
    bsdf = other.bsdf;
    other.bsdf = nullptr;
    shading_normal = other.shading_normal;
  }

  ShadingPoint& operator=(ShadingPoint&& other) {
    bsdf = other.bsdf;
    other.bsdf = nullptr;
    shading_normal = other.shading_normal;

    return *this;
  }

  ~ShadingPoint() {
    if (bsdf) delete bsdf;
  }
};

struct MediumPoint {
  Vector3f pos;
};

struct MediumScattering : MediumPoint {
  const Medium* medium = nullptr;
  Vector3f wr;
};

AJ_END
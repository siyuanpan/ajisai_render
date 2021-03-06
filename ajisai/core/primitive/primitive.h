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
#include <ajisai/core/geometry/geometry.h>
#include <ajisai/core/material/material.h>
#include <ajisai/math/spectrum.h>
#include <ajisai/math/bounds.h>

AJ_BEGIN

class AreaLight;
struct Ray;
struct PrimitiveIntersection;
struct MediumInterface;
class Geometry;

class Primitive {
 public:
  virtual ~Primitive() = default;

  virtual const AreaLight* AsLight() const noexcept = 0;

  virtual AreaLight* AsLight() noexcept = 0;

  virtual const Geometry* AsGeometry() const noexcept = 0;

  virtual bool Intersect(const Ray& ray,
                         PrimitiveIntersection* inct) const noexcept = 0;

  virtual void PostIntersect(const Ray& ray, PrimitiveIntersection* inct,
                             uint32_t id) const noexcept = 0;

  virtual bool Occlude(const Ray& ray) const noexcept = 0;

  virtual Bounds3f AABB() const noexcept = 0;

  void SetDenoise(bool denoise) noexcept { denoise_ = denoise; }

 private:
  bool denoise_ = false;
};

AJISAI_API Rc<Primitive> CreateGeometric(Rc<const Geometry> geometry,
                                         Rc<const Material> material,
                                         const MediumInterface& med,
                                         const Spectrum& emission, bool denoise,
                                         int32_t power);

AJ_END
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
#include <ajisai/math/matrix4.h>
#include <ajisai/math/bounds.h>

AJ_BEGIN

struct Ray;
struct GeometryIntersection;

struct MeshView {
  const void *position_buffer;
  size_t position_offset;
  size_t position_stride;
  size_t position_size;
  const void *index_buffer;
  size_t index_offset;
  size_t index_stride;
  size_t index_size;
};

class Geometry {
 public:
  virtual ~Geometry() = default;

  virtual bool Intersect(const Ray &ray,
                         GeometryIntersection *inct) const noexcept = 0;

  virtual void PostIntersect(const Ray &ray, GeometryIntersection *inct,
                             uint32_t id) const noexcept = 0;

  virtual bool Occlude(const Ray &ray) const noexcept = 0;

  virtual Bounds3f AABB() const noexcept = 0;

  virtual MeshView GetMeshView() const noexcept = 0;

  virtual Intersection Sample(float *pdf,
                              const Vector3f &sam) const noexcept = 0;

  virtual Intersection Sample(const Vector3f &ref, float *pdf,
                              const Vector3f &sam) const noexcept = 0;

  virtual float Pdf(const Vector3f &sample) const noexcept = 0;

  virtual float Pdf(const Vector3f &ref,
                    const Vector3f &sample) const noexcept = 0;
};

AJISAI_API Rc<Geometry> CreateQuad(const Vector3f &a, const Vector3f &b,
                                   const Vector3f &c, const Vector3f &d,
                                   const Vector2f &ta, const Vector2f &tb,
                                   const Vector2f &tc, const Vector2f &td,
                                   const Matrix4f &local2world);

AJISAI_API Rc<Geometry> CreateCube(const Matrix4f &local2world);

AJISAI_API Rc<Geometry> CreateSphere(float radius, const Vector3f &center);

AJISAI_API Rc<Geometry> CreateTwoSided(Rc<const Geometry> internal);

AJISAI_API Rc<Geometry> CreateMeshGeo(std::vector<Vector4f> &&position_and_us,
                                      std::vector<Vector4f> &&normal_and_vs,
                                      std::vector<uint32_t> &&indices);

AJ_END
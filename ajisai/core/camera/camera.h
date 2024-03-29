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
#include <ajisai/core/film.h>
#include <ajisai/core/ray.h>
#include <ajisai/math/vector3.h>

AJ_BEGIN

class Filter;

class Camera {
 public:
  virtual ~Camera() = default;

  virtual Rc<Film> CreateFilm() = 0;

  virtual Ray GenerateRay(const Vector2f& raster,
                          const Vector2f& sample) const = 0;

  virtual bool ToRaster(const Vector2f& u, const Vector3f& pos,
                        Vector3f* dir_to_camera, Vector2f* p_raster) const = 0;

  // conversion factor from image plane area density to surface area density
  virtual float ImageAreaToSurfaceArea(const Vector3f& dir_to_camera,
                                       float cos_to_camera,
                                       float distance_to_camera) const = 0;

  virtual float ImageAreaToSolidAngle(const Vector3f& direction) const = 0;
};

AJISAI_API Rc<Camera> CreateThinLensCamera(
    Rc<const Filter> filter, const Vector2f& resolution, const Vector3f& pos,
    const Vector3f& look_at, const Vector3f& up, float fov, float lens_radius,
    float focal_distance);

AJ_END
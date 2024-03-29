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
#include <ajisai/ajisai.h>
#include <ajisai/core/camera/camera.h>
#include <ajisai/core/filter/filter.h>
#include <ajisai/core/warp.h>
#include <ajisai/math/matrix4.h>

AJ_BEGIN

class ThinLensCamera : public Camera {
 public:
  explicit ThinLensCamera(Rc<const Filter> filter, const Vector2f& resolution,
                          const Vector3f& pos, const Vector3f& look_at,
                          const Vector3f& up, float fov, float lens_radius,
                          float focal_distance)
      : filter_{filter},
        resolution_{resolution},
        lens_radius_{lens_radius},
        focal_distance_{focal_distance} {
    world2camera_ = Matrix4f::lookAt(pos, look_at, up);
    camera2world_ = world2camera_.inverted();

    camera2screen_ = Matrix4f::perspectiveProjection(
        Rad<float>(fov), resolution_.AspectRatio(), 1.0f, 1000.f);
    screen2raster_ =
        Matrix4f::scaling(Vector3f{resolution_.x(), resolution_.y(), 1.f}) *
        Matrix4f::scaling(Vector3f{0.5f, -0.5f, 1.0f}) *
        Matrix4f::translation(Vector3f{1.f, -1.f, 0.0f});

    raster2camera_ = camera2screen_.inverted() * screen2raster_.inverted();
    camera2raster_ = raster2camera_.inverted();

    raster2world_ = camera2world_ * raster2camera_;
    world2Raster_ = raster2world_.inverted();
  }

  virtual Rc<Film> CreateFilm() {
    return RcNew<Film>(Vector2i{(int)resolution_[0], (int)resolution_[1]},
                       filter_.get());
  }

  virtual Ray GenerateRay(const Vector2f& raster,
                          const Vector2f& sample) const {
    Vector3f cam_coord =
        raster2camera_.transformPoint(Vector3f{raster.x(), raster.y(), 1.f});

    Ray ray{};
    ray.o = Vector3f{0.f};
    ray.d = cam_coord.normalized();

    if (lens_radius_ > 0 && focal_distance_ > 0) {
      auto lens = lens_radius_ * squareToUniformDiskConcentric(sample);

      float ft = focal_distance_ / ray.d.z();
      auto focus = ray.CalcPoint(ft);

      ray.o = Vector3f{lens.x(), lens.y(), 0.f};
      ray.d = (focus - ray.o).normalized();
    }

    return Ray{camera2world_.transformPoint(ray.o),
               camera2world_.transformVector(ray.d)};
  }

  virtual bool ToRaster(const Vector2f& u, const Vector3f& pos,
                        Vector3f* dir_to_camera,
                        Vector2f* p_raster) const override {
    Vector3f cam_pos = camera2world_.transformPoint(Vector3f{0.f});
    Vector3f cam_dir =
        camera2world_.transformVector(Vector3f{0.f, 0.f, -1.f}).normalized();

    Vector3f cam_coord = world2camera_.transformPoint(pos);
    if (lens_radius_ > 0 && focal_distance_ > 0) {
      auto lens = lens_radius_ * squareToUniformDiskConcentric(u);

      auto origin = Vector3f{lens.x(), lens.y(), 0.f};
      auto direction = (cam_coord - origin).normalized();

      float ft = focal_distance_ / direction.z();
      auto focus = origin + direction * ft;

      auto raster = camera2raster_.transformPoint(focus);
      if (!CheckRaster(raster)) return false;
      *p_raster = raster.xy();

      *dir_to_camera = camera2world_.transformPoint(origin) - pos;
      if (dot(cam_dir, -(*dir_to_camera)) <= 0.f) return false;

    } else {
      auto raster = world2Raster_.transformPoint(pos);
      if (!CheckRaster(raster)) return false;
      *p_raster = raster.xy();

      *dir_to_camera = cam_pos - pos;
      if (dot(cam_dir, -(*dir_to_camera)) <= 0.f) return false;
    }

    return true;
  }

  virtual float ImageAreaToSurfaceArea(
      const Vector3f& dir_to_camera, float cos_to_camera,
      float distance_to_camera) const override {
    Vector3f cam_dir =
        camera2world_.transformVector(Vector3f{0.f, 0.f, -1.f}).normalized();
    const float cos_at_camera = dot(cam_dir, -dir_to_camera.normalized());
    const float image_point_to_camera_dist = focal_distance_ / cos_at_camera;
    const float image_to_solid_angle_factor =
        image_point_to_camera_dist * image_point_to_camera_dist / cos_at_camera;
    const float image_to_surface_factor =
        image_to_solid_angle_factor * std::abs(cos_to_camera) /
        (distance_to_camera * distance_to_camera);

    return image_to_surface_factor;
  }

  virtual float ImageAreaToSolidAngle(
      const Vector3f& direction) const override {
    Vector3f cam_dir =
        camera2world_.transformVector(Vector3f{0.f, 0.f, -1.f}).normalized();
    const float cos_at_camera = dot(cam_dir, direction.normalized());
    const float image_point_to_camera_dist = focal_distance_ / cos_at_camera;
    const float image_to_solid_angle_factor =
        image_point_to_camera_dist * image_point_to_camera_dist / cos_at_camera;

    return image_to_solid_angle_factor;
  }

 private:
  bool CheckRaster(const Vector3f& raster) const {
    if (raster.x() < resolution_.x() && raster.x() >= 0.f &&
        raster.y() < resolution_.y() && raster.y() >= 0.f) {
      return true;
    }
    return false;
  }

  Rc<const Filter> filter_;
  Vector2f resolution_;
  float lens_radius_, focal_distance_;
  Matrix4f world2camera_, camera2world_;
  Matrix4f camera2screen_, screen2raster_;
  Matrix4f raster2camera_, camera2raster_;
  Matrix4f raster2world_, world2Raster_;
};

Rc<Camera> CreateThinLensCamera(Rc<const Filter> filter,
                                const Vector2f& resolution, const Vector3f& pos,
                                const Vector3f& look_at, const Vector3f& up,
                                float fov, float lens_radius,
                                float focal_distance) {
  return RcNew<ThinLensCamera>(filter, resolution, pos, look_at, up, fov,
                               lens_radius, focal_distance);
}

AJ_END
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
#include <ajisai/core/geometry/geometry.h>
#include <ajisai/core/geometry/helper.h>
#include <ajisai/core/warp.h>
#include <ajisai/core/coordinate_system.h>
#include <vector>
#include <algorithm>

AJ_BEGIN

class Sphere : public Geometry {
 public:
  explicit Sphere(float radius, const Vector3f &center) {
    radius_ = radius;
    center_ = center;
  }

  virtual bool Intersect(const Ray &ray,
                         GeometryIntersection *inct) const noexcept override {
    constexpr float inv_2_pi = 1.f / (2.f * Constants<float>::pi());
    constexpr float inv_pi = 1.f / Constants<float>::pi();

    const auto p = ray.o - center_;
    float B = dot(p, ray.d);
    float C = p.dot() - radius_ * radius_;
    float detSq = B * B - C;
    if (detSq > 0.f) {
      float det = std::sqrt(detSq);
      float t = -B - det;
      if (t < ray.t_max && t > ray.t_min) {
        inct->pos = ray.CalcPoint(t);
        inct->geometry_normal = (inct->pos - center_).normalized();
        inct->shading_normal = inct->geometry_normal;
        inct->uv = Vector2f{
            std::atan2(inct->geometry_normal.z(), inct->geometry_normal.x()) *
                    inv_2_pi +
                0.5f,
            std::acos(std::clamp(inct->geometry_normal.y(), -1.f, 1.f)) *
                inv_pi};
        inct->wr = -ray.d;
        return true;
      }

      t = -B + det;
      if (t < ray.t_max && t > ray.t_min) {
        inct->pos = ray.CalcPoint(t);
        inct->geometry_normal = (inct->pos - center_).normalized();
        inct->shading_normal = inct->geometry_normal;
        inct->uv = Vector2f{
            std::atan2(inct->geometry_normal.z(), inct->geometry_normal.x()) *
                    inv_2_pi +
                0.5f,
            std::acos(std::clamp(inct->geometry_normal.y(), -1.f, 1.f)) *
                inv_pi};
        inct->wr = -ray.d;
        return true;
      }
    }

    return false;
  }

  virtual void PostIntersect(const Ray &ray, GeometryIntersection *inct,
                             uint32_t id) const noexcept override {}

  virtual bool Occlude(const Ray &ray) const noexcept override {
    const auto p = ray.o - center_;
    float B = dot(p, ray.d);
    float C = p.dot() - radius_ * radius_;
    float detSq = B * B - C;
    if (detSq < 0.f) return false;

    float det = std::sqrt(detSq);
    float t1 = -B - det;
    float t2 = -B + det;

#define BETWEEN(r, t) (r.t_min <= t && t <= r.t_max)

    return BETWEEN(ray, t1) || BETWEEN(ray, t2);

#undef BETWEEN
  }

  virtual Bounds3f AABB() const noexcept override {
    Bounds3f bounds{
        center_ - Vector3f{radius_ + std::numeric_limits<float>::epsilon()},
        center_ + Vector3f{radius_ + std::numeric_limits<float>::epsilon()},
    };

    return bounds;
  }

  virtual MeshView GetMeshView() const noexcept override {
    MeshView ret{};

    return ret;
  }

  virtual Intersection Sample(float *pdf,
                              const Vector3f &sam) const noexcept override {
    constexpr float inv_2_pi = 1.f / (2.f * Constants<float>::pi());
    constexpr float inv_pi = 1.f / Constants<float>::pi();

    Vector3f unit_n = UniformSphere(Vector2f{sam.x(), sam.y()});
    *pdf = 1.f / (4 * Constants<float>::pi() * radius_ * radius_);

    Intersection inct;

    inct.pos = unit_n * radius_ + center_;
    inct.uv = Vector2f{std::atan2(unit_n.z(), unit_n.x()) * inv_2_pi + 0.5f,
                       std::acos(std::clamp(unit_n.y(), -1.f, 1.f)) * inv_pi};
    inct.geometry_normal = unit_n;
    inct.shading_normal = inct.geometry_normal;

    return inct;
  }

  virtual Intersection Sample(const Vector3f &ref, float *pdf,
                              const Vector3f &sam) const noexcept override {
    constexpr float inv_2_pi = 1.f / (2.f * Constants<float>::pi());
    constexpr float inv_pi = 1.f / Constants<float>::pi();

    Vector3f L = ref - center_;
    float d = L.length();
    if (d <= radius_) return Sample(pdf, sam);

    float sin_theta_max = radius_ / d;
    float sin_theta_max2 = sin_theta_max * sin_theta_max;
    float inv_sin_theta_max = 1.f / sin_theta_max;
    float cos_theta_max = std::sqrt(std::max(0.f, 1 - sin_theta_max2));

    const float cos_theta = (cos_theta_max - 1.f) * sam.x() + 1.f;
    const float sin_theta =
        std::sqrt(std::max(0.f, 1.f - cos_theta * cos_theta));
    float cos_alpha =
        sin_theta * sin_theta * inv_sin_theta_max +
        cos_theta * std::sqrt(std::max(0.f, 1.f - sin_theta * sin_theta *
                                                      inv_sin_theta_max *
                                                      inv_sin_theta_max));
    float sin_alpha = std::sqrt(std::max(0.f, 1.f - cos_alpha * cos_alpha));
    const float phi = sam.y() * 2.f * Constants<float>::pi();
    Vector3f dir = Vector3f{std::cos(phi) * sin_alpha,
                            std::sin(phi) * sin_alpha, cos_alpha};

    CoordinateSystem frame(L.normalized());

    Vector3f unit_n = frame.Local2World(dir);
    Intersection inct;
    inct.pos = center_ + frame.Local2World(dir) * radius_;
    inct.uv = Vector2f{std::atan2(unit_n.z(), unit_n.x()) * inv_2_pi + 0.5f,
                       std::acos(std::clamp(unit_n.y(), -1.f, 1.f)) * inv_pi};
    inct.geometry_normal = unit_n;
    inct.shading_normal = inct.geometry_normal;

    *pdf = 1.f / (2.f * Constants<float>::pi() * (1.f * cos_theta_max));

    return inct;
  }

  virtual float Pdf(const Vector3f &sample) const noexcept override {
    return 1.f / (4 * Constants<float>::pi() * radius_ * radius_);
  }

  virtual float Pdf(const Vector3f &ref,
                    const Vector3f &sample) const noexcept override {
    Vector3f L = ref - center_;
    float d = L.length();
    if (d <= radius_) return Pdf(sample);

    float sin_theta_max = radius_ / d;
    float sin_theta_max2 = sin_theta_max * sin_theta_max;
    float inv_sin_theta_max = 1.f / sin_theta_max;
    float cos_theta_max = std::sqrt(std::max(0.f, 1 - sin_theta_max2));

    return 1.f / (2.f * Constants<float>::pi() * (1.f * cos_theta_max));
  }

 private:
  float radius_;
  Vector3f center_;
};

Rc<Geometry> CreateSphere(float radius, const Vector3f &center) {
  return RcNew<Sphere>(radius, center);
}

AJ_END
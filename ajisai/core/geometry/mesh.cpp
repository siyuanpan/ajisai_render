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
#include <ajisai/core/intersection.h>
#include <ajisai/math/vector4.h>

AJ_BEGIN

class MeshGeo : public Geometry {
 public:
  explicit MeshGeo(std::vector<Vector4f> &&position_and_us,
                   std::vector<Vector4f> &&normal_and_vs,
                   std::vector<uint32_t> &&indices)
      : position_and_us_(std::move(position_and_us)),
        normal_and_vs_(std::move(normal_and_vs)),
        indices_(std::move(indices)) {
    aabb_ = {Vector3f{std::numeric_limits<float>::max()},
             Vector3f{std::numeric_limits<float>::lowest()}};
    surface_area_ = 0;

    for (size_t idx = 0; idx < indices_.size(); idx += 3) {
      int i = indices_[idx], j = indices_[idx + 1], k = indices_[idx + 2];
      auto e0 = position_and_us_[j].xyz() - position_and_us_[i].xyz();
      auto e1 = position_and_us_[k].xyz() - position_and_us_[i].xyz();
      surface_area_ += 0.5 * cross(e0, e1).length();
    }

    for (const auto &v : position_and_us_) {
      aabb_.min() = Min(aabb_.min(), v.xyz());
      aabb_.max() = Max(aabb_.max(), v.xyz());
    }
  }

  virtual bool Intersect(const Ray &ray,
                         GeometryIntersection *inct) const noexcept override {
    bool hit = false;
    for (size_t idx = 0; idx < indices_.size(); idx += 3) {
      int i = indices_[idx], j = indices_[idx + 1], k = indices_[idx + 2];
      if (IntersectWithTriangle(ray, position_and_us_[i].xyz(),
                                position_and_us_[j].xyz(),
                                position_and_us_[k].xyz(), inct)) {
        inct->pos = ray.CalcPoint(inct->t);
        inct->shading_normal =
            (1 - inct->uv.x() - inct->uv.y()) * normal_and_vs_[i].xyz() +
            inct->uv.x() * normal_and_vs_[j].xyz() +
            inct->uv.y() * normal_and_vs_[k].xyz();
        inct->uv =
            (1 - inct->uv.x() - inct->uv.y()) *
                Vector2f{position_and_us_[i].w(), normal_and_vs_[i].w()} +
            inct->uv.x() *
                Vector2f{position_and_us_[j].w(), normal_and_vs_[j].w()} +
            inct->uv.y() *
                Vector2f{position_and_us_[k].w(), normal_and_vs_[k].w()};
        inct->wr = -ray.d;
        hit = true;
      }
    }

    return hit;
  }

  virtual void PostIntersect(const Ray &ray, GeometryIntersection *inct,
                             uint32_t id) const noexcept override {
    int i = indices_[id * 3 + 0], j = indices_[id * 3 + 1],
        k = indices_[id * 3 + 2];

    inct->pos = ray.CalcPoint(inct->t);
    inct->geometry_normal =
        cross(position_and_us_[j].xyz() - position_and_us_[i].xyz(),
              position_and_us_[k].xyz() - position_and_us_[i].xyz())
            .normalized();
    inct->shading_normal =
        (1 - inct->uv.x() - inct->uv.y()) * normal_and_vs_[i].xyz() +
        inct->uv.x() * normal_and_vs_[j].xyz() +
        inct->uv.y() * normal_and_vs_[k].xyz();
    inct->uv =
        (1 - inct->uv.x() - inct->uv.y()) *
            Vector2f{position_and_us_[i].w(), normal_and_vs_[i].w()} +
        inct->uv.x() *
            Vector2f{position_and_us_[j].w(), normal_and_vs_[j].w()} +
        inct->uv.y() * Vector2f{position_and_us_[k].w(), normal_and_vs_[k].w()};
    inct->wr = -ray.d;
  }

  virtual bool Occlude(const Ray &ray) const noexcept override {
    for (size_t idx = 0; idx < indices_.size(); idx += 3) {
      int i = indices_[idx], j = indices_[idx + 1], k = indices_[idx + 2];
      if (OccludeTriangle(ray, position_and_us_[i].xyz(),
                          position_and_us_[j].xyz(),
                          position_and_us_[k].xyz())) {
        return true;
      }
    }
    return false;
  }

  virtual Bounds3f AABB() const noexcept override { return aabb_; }

  virtual MeshView GetMeshView() const noexcept override {
    MeshView ret{};
    ret.position_buffer = static_cast<const void *>(position_and_us_.data());
    ret.position_offset = 0;
    ret.position_stride = sizeof(Vector4f);
    ret.position_size = position_and_us_.size();
    ret.index_buffer = static_cast<const void *>(indices_.data());
    ret.index_offset = 0;
    ret.index_stride = 3 * sizeof(uint32_t);
    ret.index_size = indices_.size() / 3;

    return ret;
  }

  virtual Intersection Sample(float *pdf,
                              const Vector3f &sam) const noexcept override {
    Vector3f sam_new = sam;
    if (sam_new.x() + sam_new.y() > 1.f) {
      sam_new.x() = 1.f - sam_new.x();
      sam_new.y() = 1.f - sam_new.y();
    }
    Intersection inct;
    auto triangle_count = indices_.size() / 3;
    int idx = std::min(int(sam.z() * triangle_count), (int)triangle_count - 1);
    int i = indices_[idx], j = indices_[idx + 1], k = indices_[idx + 2];

    inct.pos = (1 - sam_new.x() - sam_new.y()) * position_and_us_[i].xyz() +
               sam_new.x() * position_and_us_[j].xyz() +
               sam_new.y() * position_and_us_[k].xyz();
    inct.uv =
        (1 - sam_new.x() - sam_new.y()) *
            Vector2f{position_and_us_[i].w(), normal_and_vs_[i].w()} +
        sam_new.x() * Vector2f{position_and_us_[j].w(), normal_and_vs_[j].w()} +
        sam_new.y() * Vector2f{position_and_us_[k].w(), normal_and_vs_[k].w()};
    inct.geometry_normal =
        cross(position_and_us_[j].xyz() - position_and_us_[i].xyz(),
              position_and_us_[k].xyz() - position_and_us_[i].xyz())
            .normalized();
    inct.shading_normal =
        (1 - sam_new.x() - sam_new.y()) * normal_and_vs_[i].xyz() +
        sam_new.x() * normal_and_vs_[j].xyz() +
        sam_new.y() * normal_and_vs_[k].xyz();

    *pdf = 1.f / surface_area_;

    return inct;
  }

  virtual Intersection Sample(const Vector3f &ref, float *pdf,
                              const Vector3f &sam) const noexcept override {
    return Sample(pdf, sam);
  }

  virtual float Pdf(const Vector3f &sample) const noexcept override {
    return 1.f / surface_area_;
  }

  virtual float Pdf(const Vector3f &ref,
                    const Vector3f &sample) const noexcept override {
    return Pdf(sample);
  }

 private:
  std::vector<Vector4f> position_and_us_;
  std::vector<Vector4f> normal_and_vs_;
  std::vector<uint32_t> indices_;
  Bounds3f aabb_;
  float surface_area_;
};

Rc<Geometry> CreateMeshGeo(std::vector<Vector4f> &&position_and_us,
                           std::vector<Vector4f> &&normal_and_vs,
                           std::vector<uint32_t> &&indices) {
  return RcNew<MeshGeo>(std::move(position_and_us), std::move(normal_and_vs),
                        std::move(indices));
}

AJ_END
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
#include <vector>

AJ_BEGIN

class Cube : public Geometry {
 public:
  struct Args {
    Matrix4f local2world;
  };

  explicit Cube(const Args &args) {
    const Vector3f verts[6][4] = {
        {{-0.5f, -0.5f, -0.5f},
         {-0.5f, -0.5f, 0.5f},
         {0.5f, -0.5f, 0.5f},
         {0.5f, -0.5f, -0.5f}},
        {{-0.5f, 0.5f, 0.5f},
         {-0.5f, 0.5f, -0.5f},
         {0.5f, 0.5f, -0.5f},
         {0.5f, 0.5f, 0.5f}},
        {{-0.5f, 0.5f, -0.5f},
         {-0.5f, -0.5f, -0.5f},
         {0.5f, -0.5f, -0.5f},
         {0.5f, 0.5f, -0.5f}},
        {{0.5f, 0.5f, 0.5f},
         {0.5f, -0.5f, 0.5f},
         {-0.5f, -0.5f, 0.5f},
         {-0.5f, 0.5f, 0.5f}},
        {{-0.5f, 0.5f, 0.5f},
         {-0.5f, -0.5f, 0.5f},
         {-0.5f, -0.5f, -0.5f},
         {-0.5f, 0.5f, -0.5f}},
        {{0.5f, 0.5f, -0.5f},
         {0.5f, -0.5f, -0.5f},
         {0.5f, -0.5f, 0.5f},
         {0.5f, 0.5f, 0.5f}},
    };

    const Vector2f uvs[] = {
        {0.0f, 0.0f}, {1.0f, 0.0f}, {1.0f, 1.0f}, {0.0f, 1.0f}};

    for (int i = 0; i < 6; ++i) {
      uint32_t idx = positions_.size();
      tris_.emplace_back(idx, idx + 2, idx + 1);
      tris_.emplace_back(idx, idx + 3, idx + 2);

      for (int j = 0; j < 4; ++j) {
        positions_.emplace_back(args.local2world.transformPoint(verts[i][j]));
        uv_.emplace_back(uvs[j]);
      }
    }
  }

  virtual bool Intersect(const Ray &ray,
                         GeometryIntersection *inct) const noexcept override {
    bool hit = false;
    for (auto [i, j, k] : tris_) {
      if (IntersectWithTriangle(ray, positions_[i], positions_[j],
                                positions_[k], inct)) {
        inct->pos = ray.CalcPoint(inct->t);
        inct->shading_normal = inct->geometry_normal;
        inct->uv = (1 - inct->uv.x() - inct->uv.y()) * uv_[i] +
                   inct->uv.x() * uv_[j] + inct->uv.y() * uv_[k];
        inct->wr = -ray.d;
        hit = true;
      }
    }

    return hit;
  }

  virtual void PostIntersect(const Ray &ray, GeometryIntersection *inct,
                             uint32_t id) const noexcept override {
    auto [i, j, k] = tris_[id];

    inct->pos = ray.CalcPoint(inct->t);
    inct->geometry_normal =
        cross(positions_[j] - positions_[i], positions_[k] - positions_[i])
            .normalized();
    inct->shading_normal = inct->geometry_normal;
    inct->uv = (1 - inct->uv.x() - inct->uv.y()) * uv_[i] +
               inct->uv.x() * uv_[j] + inct->uv.y() * uv_[k];
    inct->wr = -ray.d;
  }

  virtual bool Occlude(const Ray &ray) const noexcept override {
    for (auto [i, j, k] : tris_) {
      if (OccludeTriangle(ray, positions_[i], positions_[j], positions_[k])) {
        return true;
      }
    }
    return false;
  }

  virtual Bounds3f AABB() const noexcept override {
    Bounds3f bounds{Vector3f{std::numeric_limits<float>::max()},
                    Vector3f{std::numeric_limits<float>::lowest()}};

    for (const auto &v : positions_) {
      bounds.min() = Min(bounds.min(), v);
      bounds.max() = Max(bounds.max(), v);
    }

    return bounds;
  }

  virtual MeshView GetMeshView() const noexcept override {
    MeshView ret{};
    ret.position_buffer = static_cast<const void *>(positions_.data());
    ret.position_offset = 0;
    ret.position_stride = sizeof(Vector3f);
    ret.position_size = positions_.size();
    ret.index_buffer = static_cast<const void *>(tris_.data());
    ret.index_offset = 0;
    ret.index_stride = sizeof(std::tuple<uint32_t, uint32_t, uint32_t>);
    ret.index_size = tris_.size();

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
    int idx = std::min(int(sam.z() * tris_.size()), (int)tris_.size() - 1);
    auto [a, b, c] = tris_[idx];

    inct.pos = (1 - sam_new.x() - sam_new.y()) * positions_[a] +
               sam_new.x() * positions_[b] + sam_new.y() * positions_[c];
    inct.uv = (1 - sam_new.x() - sam_new.y()) * uv_[a] + sam_new.x() * uv_[b] +
              sam_new.y() * uv_[c];
    inct.geometry_normal =
        cross(positions_[b] - positions_[a], positions_[c] - positions_[a])
            .normalized();
    inct.shading_normal = inct.geometry_normal;

    *pdf = 1.f /
           (12.f *
            cross(positions_[b] - positions_[a], positions_[c] - positions_[a])
                .length() *
            0.5f);

    return inct;
  }

  virtual Intersection Sample(const Vector3f &ref, float *pdf,
                              const Vector3f &sam) const noexcept override {
    return Sample(pdf, sam);
  }

  virtual float Pdf(const Vector3f &sample) const noexcept override {
    auto [a, b, c] = tris_[0];
    return 1.f /
           (12.f *
            cross(positions_[b] - positions_[a], positions_[c] - positions_[a])
                .length() *
            0.5f);
  }

  virtual float Pdf(const Vector3f &ref,
                    const Vector3f &sample) const noexcept override {
    return Pdf(sample);
  }

 private:
  std::vector<Vector3f> positions_;
  std::vector<std::tuple<uint32_t, uint32_t, uint32_t>> tris_;
  std::vector<Vector2f> uv_;
};

Rc<Geometry> CreateCube(const Matrix4f &local2world) {
  return RcNew<Cube>(Cube::Args{local2world});
}

AJ_END
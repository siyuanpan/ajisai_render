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
        inct->uv = (1 - inct->uv.x() - inct->uv.y()) * uv_[i] +
                   inct->uv.x() * uv_[j] + inct->uv.y() * uv_[k];
        inct->wr = -ray.d;
        hit = true;
      }
    }

    return hit;
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
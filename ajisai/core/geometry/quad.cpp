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

AJ_BEGIN

class Quad : public Geometry {
 public:
  struct Args {
    Vector3f a, b, c, d;
    Vector2f ta, tb, tc, td;
    Matrix4f local2world;
  };

  explicit Quad(const Args &args)
      : ta_(args.ta), tb_(args.tb), tc_(args.tc), td_(args.td) {
    a_ = args.local2world.transformPoint(args.a);
    b_ = args.local2world.transformPoint(args.b);
    c_ = args.local2world.transformPoint(args.c);
    d_ = args.local2world.transformPoint(args.d);
  }

  virtual bool Intersect(const Ray &ray,
                         GeometryIntersection *inct) const noexcept override {
    if (IntersectWithTriangle(ray, a_, b_, c_, inct)) {
      inct->pos = ray.CalcPoint(inct->t);
      inct->uv = (1 - inct->uv.x() - inct->uv.y()) * ta_ + inct->uv.x() * tb_ +
                 inct->uv.y() * tc_;
      inct->wr = -ray.d;
      return true;
    }
    if (IntersectWithTriangle(ray, a_, c_, d_, inct)) {
      inct->pos = ray.CalcPoint(inct->t);
      inct->uv = (1 - inct->uv.x() - inct->uv.y()) * ta_ + inct->uv.x() * tc_ +
                 inct->uv.y() * td_;
      inct->wr = -ray.d;
      return true;
    }
    return false;
  }

 private:
  Vector3f a_, b_, c_, d_;
  Vector2f ta_, tb_, tc_, td_;
};

Rc<Geometry> CreateQuad(const Vector3f &a, const Vector3f &b, const Vector3f &c,
                        const Vector3f &d, const Vector2f &ta,
                        const Vector2f &tb, const Vector2f &tc,
                        const Vector2f &td, const Matrix4f &local2world) {
  return RcNew<Quad>(Quad::Args{a, b, c, d, ta, tb, tc, td, local2world});
}

AJ_END
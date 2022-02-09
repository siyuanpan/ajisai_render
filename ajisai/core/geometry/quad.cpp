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

    auto abc = cross(b_ - a_, c_ - a_).length() * 0.5f;
    auto acd = cross(c_ - a_, d_ - a_).length() * 0.5f;
    surface_area_ = abc + acd;
    sample_prob_ = abc / surface_area_;
  }

  virtual bool Intersect(const Ray &ray,
                         GeometryIntersection *inct) const noexcept override {
    if (IntersectWithTriangle(ray, a_, b_, c_, inct)) {
      inct->pos = ray.CalcPoint(inct->t);
      inct->shading_normal = inct->geometry_normal;
      inct->uv = (1 - inct->uv.x() - inct->uv.y()) * ta_ + inct->uv.x() * tb_ +
                 inct->uv.y() * tc_;
      inct->wr = -ray.d;
      return true;
    }
    if (IntersectWithTriangle(ray, a_, c_, d_, inct)) {
      inct->pos = ray.CalcPoint(inct->t);
      inct->shading_normal = inct->geometry_normal;
      inct->uv = (1 - inct->uv.x() - inct->uv.y()) * ta_ + inct->uv.x() * tc_ +
                 inct->uv.y() * td_;
      inct->wr = -ray.d;
      return true;
    }
    return false;
  }

  virtual bool Occlude(const Ray &ray) const noexcept override {
    return OccludeTriangle(ray, a_, b_, c_) || OccludeTriangle(ray, a_, c_, d_);
  }

  virtual Bounds3f AABB() const noexcept override {
    Bounds3f bounds{Vector3f{std::numeric_limits<float>::max()},
                    Vector3f{std::numeric_limits<float>::lowest()}};

    bounds.min() = Min(bounds.min(), a_);
    bounds.max() = Max(bounds.max(), a_);

    bounds.min() = Min(bounds.min(), b_);
    bounds.max() = Max(bounds.max(), b_);

    bounds.min() = Min(bounds.min(), c_);
    bounds.max() = Max(bounds.max(), c_);

    bounds.min() = Min(bounds.min(), d_);
    bounds.max() = Max(bounds.max(), d_);

    return bounds;
  }

  virtual Intersection Sample(float *pdf,
                              const Vector3f &sam) const noexcept override {
    Vector3f sam_new = sam;
    if (sam_new.x() + sam_new.y() > 1.f) {
      sam_new.x() = 1.f - sam_new.x();
      sam_new.y() = 1.f - sam_new.y();
    }
    Intersection inct;

    if (sam.z() < sample_prob_) {
      inct.pos = (1 - sam_new.x() - sam_new.y()) * a_ + sam_new.x() * b_ +
                 sam_new.y() * c_;
      inct.uv = (1 - sam_new.x() - sam_new.y()) * ta_ + sam_new.x() * tb_ +
                sam_new.y() * tc_;
      inct.geometry_normal = cross(b_ - a_, c_ - a_).normalized();
      inct.shading_normal = inct.geometry_normal;
    } else {
      inct.pos = (1 - sam_new.x() - sam_new.y()) * a_ + sam_new.x() * c_ +
                 sam_new.y() * d_;
      inct.uv = (1 - sam_new.x() - sam_new.y()) * ta_ + sam_new.x() * tc_ +
                sam_new.y() * td_;
      inct.geometry_normal = cross(c_ - a_, d_ - a_).normalized();
      inct.shading_normal = inct.geometry_normal;
    }

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
  Vector3f a_, b_, c_, d_;
  Vector2f ta_, tb_, tc_, td_;
  float surface_area_;
  float sample_prob_;
};

Rc<Geometry> CreateQuad(const Vector3f &a, const Vector3f &b, const Vector3f &c,
                        const Vector3f &d, const Vector2f &ta,
                        const Vector2f &tb, const Vector2f &tc,
                        const Vector2f &td, const Matrix4f &local2world) {
  return RcNew<Quad>(Quad::Args{a, b, c, d, ta, tb, tc, td, local2world});
}

AJ_END
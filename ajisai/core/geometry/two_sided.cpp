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
#include <ajisai/core/intersection.h>

AJ_BEGIN

class TwoSided : public Geometry {
 public:
  explicit TwoSided(Rc<const Geometry> internal) : internal_(internal) {}

  virtual bool Intersect(const Ray &ray,
                         GeometryIntersection *inct) const noexcept override {
    if (!internal_->Intersect(ray, inct)) return false;

    const bool backface = dot(inct->geometry_normal, inct->wr) < 0;
    if (backface) {
      inct->geometry_normal *= -1;
      inct->shading_normal *= -1;
    }
    return true;
  }

  virtual bool Occlude(const Ray &ray) const noexcept override {
    return internal_->Occlude(ray);
  }

  virtual Intersection Sample(float *pdf,
                              const Vector3f &sam) const noexcept override {
    Vector3f sam_new = sam;
    const bool backface = sam_new.x() < 0.5f;
    if (backface) {
      sam_new.x() = 2 * sam.x();
    } else {
      sam_new.x() = 2 * (sam.x() - 0.5f);
    }

    Intersection ict = internal_->Sample(pdf, sam_new);
    *pdf = *pdf * 0.5;
    if (backface) {
      ict.geometry_normal *= -1;
      ict.shading_normal *= -1;
    }

    return ict;
  }

  virtual Intersection Sample(const Vector3f &ref, float *pdf,
                              const Vector3f &sam) const noexcept override {
    Vector3f sam_new = sam;
    const bool backface = sam_new.x() < 0.5f;
    if (backface) {
      sam_new.x() = 2 * sam.x();
    } else {
      sam_new.x() = 2 * (sam.x() - 0.5f);
    }

    Intersection ict = internal_->Sample(ref, pdf, sam_new);
    *pdf = *pdf * 0.5;
    if (backface) {
      ict.geometry_normal *= -1;
      ict.shading_normal *= -1;
    }

    return ict;
  }

  virtual float Pdf(const Vector3f &sample) const noexcept override {
    const float internal_pdf = internal_->Pdf(sample);
    return internal_pdf * 0.5f;
  }

  virtual float Pdf(const Vector3f &ref,
                    const Vector3f &sample) const noexcept override {
    const float internal_pdf = internal_->Pdf(ref, sample);
    return internal_pdf * 0.5f;
  }

 private:
  Rc<const Geometry> internal_;
};

Rc<Geometry> CreateTwoSided(Rc<const Geometry> internal) {
  return RcNew<TwoSided>(std::move(internal));
}

AJ_END
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
#include <ajisai/core/aggregate/aggregate.h>
#include <ajisai/core/ray.h>
#include <ajisai/core/intersection.h>

AJ_BEGIN

class NativeAggregate : public Aggregate {
 public:
  virtual void Build(const std::vector<Rc<Primitive>>& primitives) override {
    primitives_.assign(primitives.begin(), primitives.end());
  }

  virtual bool Intersect(const Ray& ray,
                         PrimitiveIntersection* inct) const noexcept override {
    Ray r = ray;
    bool ret = false;
    for (auto& primitive : primitives_) {
      if (primitive->Intersect(r, inct)) {
        r.t_max = inct->t;
        ret = true;
      }
    }

    return ret;
  }

  virtual bool Occlude(const Ray& ray) const noexcept override {
    for (auto& primitive : primitives_) {
      if (primitive->Occlude(ray)) {
        return true;
      }
    }
    return false;
  }

 private:
  std::vector<Rc<const Primitive>> primitives_;
};

Rc<Aggregate> CreateNativeAggregate() { return RcNew<NativeAggregate>(); }

AJ_END
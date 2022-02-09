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
#include <ajisai/core/primitive/primitive.h>
#include <ajisai/core/light/area_light.h>
#include <ajisai/core/intersection.h>
#include <ajisai/core/medium/medium.h>

AJ_BEGIN

class GeometricPrimitive : public Primitive {
 public:
  explicit GeometricPrimitive(Rc<const Geometry> geometry,
                              Rc<const Material> material,
                              const MediumInterface& med,
                              const Spectrum& emission, bool denoise,
                              int32_t power) {
    geometry_ = std::move(geometry);
    material_ = std::move(material);
    medium_interface_ = med;
    SetDenoise(denoise);

    if (!emission.IsBlack()) {
      area_light_ = BoxNew<AreaLight>(geometry_.get(), emission, power);
    }
  }

  virtual const AreaLight* AsLight() const noexcept override {
    return area_light_.get();
  }

  virtual AreaLight* AsLight() noexcept override { return area_light_.get(); }

  virtual bool Intersect(const Ray& ray,
                         PrimitiveIntersection* inct) const noexcept override {
    if (!geometry_->Intersect(ray, inct)) return false;
    inct->primitive = this;
    inct->material = material_.get();
    inct->medium_in = medium_interface_.in.get();
    inct->medium_out = medium_interface_.out.get();

    return true;
  }

  virtual bool Occlude(const Ray& ray) const noexcept override {
    return geometry_->Occlude(ray);
  }

  virtual Bounds3f AABB() const noexcept override { return geometry_->AABB(); }

 private:
  Rc<const Geometry> geometry_;
  Rc<const Material> material_;
  MediumInterface medium_interface_;
  Box<AreaLight> area_light_;
};

Rc<Primitive> CreateGeometric(Rc<const Geometry> geometry,
                              Rc<const Material> material,
                              const MediumInterface& med,
                              const Spectrum& emission, bool denoise,
                              int32_t power) {
  return RcNew<GeometricPrimitive>(std::move(geometry), std::move(material),
                                   med, emission, denoise, power);
}

AJ_END
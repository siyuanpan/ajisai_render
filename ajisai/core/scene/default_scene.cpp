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
#include <ajisai/core/scene/scene.h>
#include <ajisai/core/light/area_light.h>

AJ_BEGIN

class DefaultScene : public Scene {
 public:
  explicit DefaultScene(const SceneArgs& args) {
    for (auto& primitive : args.primitives) {
      if (auto light = primitive->AsLight()) {
        lights_.push_back(light);
      }
    }
    primitives_ = args.primitives;
    aggregate_ = args.aggregate;
  }

  virtual bool Intersect(const Ray& ray,
                         PrimitiveIntersection* inct) const noexcept override {
    return aggregate_->Intersect(ray, inct);
  }

  virtual bool Occlude(const Ray& ray) const noexcept override {
    return aggregate_->Occlude(ray);
  }

  virtual std::span<const Light* const> Lights() const noexcept override {
    const auto ptr = lights_.data();
    return std::span<const Light* const>(ptr, lights_.size());
  }

 private:
  std::vector<Rc<Primitive>> primitives_;
  Rc<Aggregate> aggregate_;
  std::vector<Light*> lights_;
};

Rc<Scene> CreateDefaultScene(const SceneArgs& args) {
  return RcNew<DefaultScene>(args);
}

AJ_END
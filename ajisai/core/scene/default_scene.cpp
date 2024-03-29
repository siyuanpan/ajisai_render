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
#include <ajisai/core/light/env_light.h>

AJ_BEGIN

class DefaultScene : public Scene {
 public:
  explicit DefaultScene(const SceneArgs& args) {
    Bounds3f world_bound{Vector3f{std::numeric_limits<float>::max()},
                         Vector3f{std::numeric_limits<float>::lowest()}};
    for (auto& primitive : args.primitives) {
      if (auto light = primitive->AsLight()) {
        lights_ptr_.push_back(light);
      }
      world_bound = Join(world_bound, primitive->AABB());
    }

    for (auto& light : args.lights) {
      if (auto env_light = light->AsEnv()) {
        env_light_ = const_cast<EnvLight*>(env_light);
      }

      lights_ptr_.push_back(light.get());
      light->Process(world_bound);
    }

    pdf_table_.resize(lights_ptr_.size(), 1.f);
    const float sum =
        std::accumulate(pdf_table_.begin(), pdf_table_.end(), 0.f);
    float ratio = 1.f / sum;
    for (auto& p : pdf_table_) p *= ratio;
    light_selector_.SetFunction(pdf_table_.data(), pdf_table_.size());

    primitives_ = args.primitives;
    lights_ = args.lights;
    aggregate_ = args.aggregate;
    world_bound_ = world_bound;
  }

  virtual bool Intersect(const Ray& ray,
                         PrimitiveIntersection* inct) const noexcept override {
    return aggregate_->Intersect(ray, inct);
  }

  virtual bool Occlude(const Ray& ray) const noexcept override {
    return aggregate_->Occlude(ray);
  }

  virtual std::span<const Light* const> Lights() const noexcept override {
    const auto ptr = lights_ptr_.data();
    return std::span<const Light* const>(ptr, lights_ptr_.size());
  }

  virtual SceneSampleLightResult SampleLight(
      const float u) const noexcept override {
    const size_t idx = light_selector_.SampleDiscrete(u);
    assert(u <= idx && idx < lights_ptr_.size());

    SceneSampleLightResult ret{};
    ret.light = lights_ptr_[idx];
    ret.pdf = pdf_table_[idx];
    return ret;
  }

  virtual const EnvLight* GetEnvLight() const noexcept override {
    return env_light_;
  }

  virtual Bounds3f WorldBound() const noexcept override { return world_bound_; }

 private:
  std::vector<Rc<Primitive>> primitives_;
  std::vector<Rc<Light>> lights_;
  Rc<Aggregate> aggregate_;
  std::vector<Light*> lights_ptr_;
  EnvLight* env_light_ = nullptr;
  Bounds3f world_bound_;
  Distribution light_selector_;
  std::vector<float> pdf_table_;
};

Rc<Scene> CreateDefaultScene(const SceneArgs& args) {
  return RcNew<DefaultScene>(args);
}

AJ_END
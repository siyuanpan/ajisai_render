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
#pragma once
#include <ajisai/ajisai.h>
#include <ajisai/core/aggregate/aggregate.h>
#include <ajisai/core/light/light.h>
#include <span>

AJ_BEGIN

struct Ray;
struct PrimitiveIntersection;
class Light;
class EnvLight;

struct SceneArgs {
  std::vector<Rc<Primitive>> primitives;
  std::vector<Rc<Light>> lights;
  Rc<Aggregate> aggregate;
};

struct SceneSampleLightResult {
  Light* light;
  float pdf;
};

class Scene {
 public:
  virtual ~Scene() = default;

  virtual bool Intersect(const Ray& ray,
                         PrimitiveIntersection* inct) const noexcept = 0;

  virtual bool Occlude(const Ray& ray) const noexcept = 0;

  virtual std::span<const Light* const> Lights() const noexcept = 0;

  virtual SceneSampleLightResult SampleLight(const float u) const noexcept = 0;

  virtual const EnvLight* GetEnvLight() const noexcept = 0;

  virtual Bounds3f WorldBound() const noexcept = 0;
};

AJISAI_API Rc<Scene> CreateDefaultScene(const SceneArgs&);

AJ_END
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
#ifndef AJISAI_CORE_SCENE_H_
#define AJISAI_CORE_SCENE_H_

#include <Ajisai/Core/Camera.h>
#include <Ajisai/Core/Light.h>
#include <Ajisai/Core/Mesh.h>

// #include <cmath>

namespace Ajisai::Core {

class Scene {
 public:
  void AddMesh(const std::shared_ptr<Mesh>& mesh) {
    meshes.emplace_back(mesh);
    if (mesh->IsEmitter()) {
      auto meshLights = mesh->GetLights();
      for (auto light : meshLights) {
        lights.emplace_back(light);
      }
    }
  }

  const Mesh& GetMesh(size_t i) const { return *meshes[i]; }

  bool Intersect(const Ray& ray, Intersection* intersection) const {
    bool hit = false;
    for (int i = 0; i < meshes.size(); ++i) {
      auto& mesh = meshes[i];
      auto ret = mesh->Intersect(ray, intersection);
      // if (ret) {
      //   intersection->meshId = i;
      // }
      hit |= ret;
    }

    return hit;
  }

  const AreaLight* SampleOneLight(/*Math::Vector2f& sample*/ float u,
                                  float* pdf) const {
    if (lights.empty()) return nullptr;

    float light_pdf = 1.f / lights.size();

    size_t index =
        std::min(size_t(u * (float)lights.size()), (size_t)lights.size() - 1);

    // sample.x() = (sample.x() - index * light_pdf) * lights.size();
    auto light = lights[index].get();
    *pdf = light_pdf;
    return light;
  }

 protected:
  std::vector<std::shared_ptr<const Mesh>> meshes;
  std::vector<std::shared_ptr<AreaLight>> lights;

 private:
};

}  // namespace Ajisai::Core

#endif
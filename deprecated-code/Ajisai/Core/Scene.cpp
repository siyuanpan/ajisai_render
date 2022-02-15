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

#include <Ajisai/Core/Scene.h>

namespace Ajisai::Core {
bool Scene::Intersect(const Ray& ray, DifferentialGeom* diffGeom) const {
  bool hit = false;
  if (accel) {
    hit = accel->Intersect(ray, diffGeom);
  } else {
    assert(false);
  }

  return hit;
}

void Scene::PostIntersect(const Ray& ray, DifferentialGeom* diffGeom) const {
  auto& mesh = this->GetMesh(diffGeom->_geomID);
  if (mesh.IsEmitter()) {
    diffGeom->_areaLight = mesh.GetLight(diffGeom->_primID).get();
  }

  Triangle triangle{};
  mesh.GetTriangle(diffGeom->_primID, &triangle);
  diffGeom->_position = ray.Point(diffGeom->_dist);
  diffGeom->_normal = triangle.lerpNormal(diffGeom->_uv);
  diffGeom->_texcoord = triangle.lerpTexCoord(diffGeom->_uv);
  mesh.GetMaterial()->ComputeScatteringFunction(diffGeom);
}
}  // namespace Ajisai::Core

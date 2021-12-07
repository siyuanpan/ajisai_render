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

#include <Ajisai/Core/Geometry.h>
#include <Ajisai/Core/Mesh.h>
#include <Ajisai/Core/Scene.h>

namespace Ajisai::Core {

Math::Spectrum Intersection::Le(const Math::Vector3f& wo) const {
  return mesh->Le(wo);
}

Math::Spectrum SurfaceInteraction::Le(const Math::Vector3f& wo) const {
  return mesh->Le(wo);
}

[[nodiscard]] bool VisibilityTester::visible(const Scene& scene) const {
  return !scene.Occlude(shadowRay);
}
[[nodiscard]] Math::Spectrum VisibilityTester::Tr(const Scene& scene) const {
  return scene.Occlude(shadowRay) ? Math::Spectrum(0) : Math::Spectrum(1);
}

}  // namespace Ajisai::Core
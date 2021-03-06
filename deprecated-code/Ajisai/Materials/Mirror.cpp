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

#include <Ajisai/Materials/Mirror.h>

namespace Ajisai::Materials {

MirrorMaterial::MirrorMaterial(const Math::Color3<float> c) : color(c) {}

void MirrorMaterial::ComputeScatteringFunction(Core::SurfaceInteraction* si,
                                               Core::TransportMode mode) const {
  si->bsdf = Util::Ptr<Core::BSDF>(new Core::BSDF(
      si->Ng, si->Ns));  // std::make_shared<Core::BSDF>(si->Ng, si->Ns);
  si->bsdf->add(std::make_shared<Core::SpecularReflection>(
      color, new Core::FresnelNoOp()));
}

void MirrorMaterial::ComputeScatteringFunction(
    Core::DifferentialGeom* diffGeom) const {
  diffGeom->_bsdf = Util::Ptr<Core::BSDF>(
      new Core::BSDF(diffGeom->_geomNormal, diffGeom->_normal));
  diffGeom->_bsdf->add(std::make_shared<Core::SpecularReflection>(
      color, new Core::FresnelNoOp()));
}

}  // namespace Ajisai::Materials
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

#ifndef AJISAI_CORE_LIGHT
#define AJISAI_CORE_LIGHT

#include <Ajisai/Core/Geometry.h>
#include <Ajisai/Core/Mesh.h>
#include <Ajisai/Math/Math.h>

namespace Ajisai::Core {
class AreaLight {
 public:
  AreaLight(Mesh* mesh, int triId, const Math::Spectrum& radiance)
      : mesh(mesh), triId(triId), radiance(radiance) {
    mesh->GetTriangle(triId, &triangle);
  }

  void SampleLi(Math::Vector2f sample, const Math::Vector3f& p,
                LightSamplingRecord& lRec) const {
    Triangle triangle;
    mesh->GetTriangle(triId, &triangle);
    if (sample.x() + sample.y() > 1) {
      sample.x() = 1 - sample.x();
      sample.y() = 1 - sample.y();
    }
    auto uv = sample;
    auto x = triangle.lerpVert(uv);
    auto pdf = 1.f / triangle.area();
    auto normal = triangle.lerpNormal(uv);
    auto wi = x - p;
    // std::cout << "x : " << x[0] << " " << x[1] << " " << x[2] << std::endl;
    // std::cout << "p : " << p[0] << " " << p[1] << " " << p[2] << std::endl;
    // std::cout << "wi : " << wi[0] << " " << wi[1] << " " << wi[2] <<
    // std::endl; std::cout << "pdf: " << pdf << std::endl;
    auto dist2 = Math::dot(wi, wi);
    auto dist = wi.length();
    // wi /= dist;
    wi = wi.normalized();

    lRec.Li = radiance;  //{8.5f, 6.f, 2.f};  //
                         // Math::Spectrum(17.f, 12.f, 4.f);  // color;
    lRec.wi = wi;
    lRec.pdf = dist2 / (-Math::dot(lRec.wi, normal)) * pdf;
    lRec.normal = normal;
    lRec.shadowRay = Ray(
        x, -wi,
        std::numeric_limits<float>::epsilon() / std::abs(Math::dot(wi, normal)),
        dist * 0.99);
  }

  float pdfLi(Intersection its, const Math::Vector3f& wi) const {
    Intersection _its;
    Ray ray(its.p, wi);
    if (!triangle.Intersect(ray, &_its)) {
      return 0.f;
    }
    // std::cout << "this\n";
    // auto tmp =
    //     _its.t * _its.t / (-Math::dot(wi, _its.Ng)) * (1.f /
    //     triangle.area());
    // std::cout << "pdf : " << tmp << std::endl;
    return _its.t * _its.t / (-Math::dot(wi, _its.Ng)) *
           (1.f / triangle.area());
  }

  Mesh* mesh;
  int triId;
  Math::Spectrum radiance;
  Triangle triangle;
};
}  // namespace Ajisai::Core

#endif
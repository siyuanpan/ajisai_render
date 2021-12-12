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

inline float cosTheta(const Math::Vector3f& w) { return w.z(); }
inline float absCosTheta(const Math::Vector3f& w) {
  return std::abs(cosTheta(w));
}
class AreaLight {
 public:
  AreaLight(Mesh* mesh, int triId, const Math::Spectrum& radiance)
      : mesh(mesh), triId(triId), radiance(radiance) {
    mesh->GetTriangle(triId, &triangle);
    frame = CoordinateSystem(triangle.Ng);
  }

  Math::Spectrum Li(const Math::Vector3f& wo) const {
    if (Math::dot(wo, triangle.Ng) < 0) return Math::Spectrum{0.f};
    return radiance;
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

  void Sample_Li(const Math::Vector2f& u, const Intersect& ref,
                 LightSamplingRecord& lRec, VisibilityTester& tester) const {
    auto uv = u;
    if (uv.x() + uv.y() > 1) {
      uv.x() = 1 - uv.x();
      uv.y() = 1 - uv.y();
    }

    auto x = triangle.lerpVert(uv);
    auto pdf = 1.f / triangle.area();
    auto normal = triangle.lerpNormal(uv);
    auto wi = x - ref.p;

    auto dist2 = Math::dot(wi, wi);
    auto dist = wi.length();
    // wi /= dist;
    wi = wi.normalized();

    lRec.Li = radiance;  //{8.5f, 6.f, 2.f};  //
                         // Math::Spectrum(17.f, 12.f, 4.f);  // color;
    lRec.wi = wi;
    lRec.pdf = dist2 / (-Math::dot(lRec.wi, normal)) * pdf;
    lRec.normal = normal;
    // lRec.shadowRay
    tester.shadowRay = Ray(
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

    return _its.t * _its.t / (-Math::dot(wi, _its.Ng)) *
           (1.f / triangle.area());
  }

  bool isFinite() const { return true; }

  bool isDelta() const { return false; }

  void Sample_Le(const Math::Vector2f& u1, const Math::Vector2f& u2, Ray* ray,
                 Math::Vector3f& nLight, float* pdfPos, float* pdfDir,
                 Math::Spectrum& E) const {
    Triangle triangle;
    mesh->GetTriangle(triId, &triangle);
    auto u = u1;
    if (u.x() + u.y() > 1) {
      u.x() = 1 - u.x();
      u.y() = 1 - u.y();
    }
    auto uv = u;
    auto x = triangle.lerpVert(uv);
    nLight = triangle.lerpNormal(uv);
    *pdfPos = 1.f / triangle.area();

    auto wi = squareToCosineHemisphere(u2);
    *pdfDir = absCosTheta(wi) / Math::Constants<float>::pi();
    auto wiW = frame.toWorld(wi);
    *ray = Ray(x, wiW);
    E = Math::dot(wiW, triangle.Ng) > 0 ? radiance * absCosTheta(wi)
                                        : Math::Spectrum{};
  }

  void Pdf_Le(const Ray& ray, float* pdfPos, float* pdfDir) const {
    *pdfPos = 1 / triangle.area();
    *pdfDir = std::fmax(
        0.0f, Math::dot(triangle.Ng, ray.d) / Math::Constants<float>::pi());
  }

  Mesh* mesh;
  int triId;
  Math::Spectrum radiance;
  Triangle triangle;
  CoordinateSystem frame;
};
}  // namespace Ajisai::Core

#endif
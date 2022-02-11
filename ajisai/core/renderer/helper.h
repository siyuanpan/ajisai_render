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
#include <ajisai/core/scene/scene.h>
#include <ajisai/core/light/area_light.h>
#include <ajisai/core/intersection.h>
#include <ajisai/core/sampler/sampler.h>
#include <ajisai/math/spectrum.h>

AJ_BEGIN

inline float BalanceHeuristic(float pdfa, float pdfb) {
  return pdfa / (pdfa + pdfb);
}

inline float PowerHeuristic(float pdfa, float pdfb) {
  pdfa *= pdfa;
  pdfb *= pdfb;
  return pdfa / (pdfa + pdfb);
}

inline Spectrum MISSampleAreaLight(const Scene* scene, const AreaLight* light,
                                   const PrimitiveIntersection& inct,
                                   const ShadingPoint& sp, Sampler* sampler) {
  const auto light_sample = light->Sample(inct.pos, sampler);
  if (light_sample.radiance.IsBlack() || light_sample.pdf <= 0.f) return {};

  const float shadow_ray_len =
      (light_sample.pos - inct.pos).length() - Ray::Eps();
  if (shadow_ray_len < Ray::Eps()) return {};

  const Vector3f inct2light = (light_sample.pos - inct.pos).normalized();
  const Ray shadow_ray{inct.pos, inct2light, Ray::Eps(),
                       0.99f * shadow_ray_len};
  if (scene->Occlude(shadow_ray)) return {};

  auto med = inct.GetMedium(inct2light);
  const auto bsdf_f =
      sp.bsdf->EvalAll(inct2light, inct.wr, TransMode::Radiance);
  if (bsdf_f.IsBlack()) return {};

  const Spectrum f = med->Tr(light_sample.pos, inct.pos, sampler) *
                     light_sample.radiance * bsdf_f *
                     std::abs(dot(inct2light, inct.geometry_normal));
  const float bsdf_pdf = sp.bsdf->PdfAll(inct2light, inct.wr);

  return f / light_sample.pdf * PowerHeuristic(light_sample.pdf, bsdf_pdf);
}

inline Spectrum MISSampleLight(const Scene* scene, const Light* light,
                               const PrimitiveIntersection& inct,
                               const ShadingPoint& sp, Sampler* sampler) {
  if (auto area_light = light->AsArea()) {
    return MISSampleAreaLight(scene, area_light, inct, sp, sampler);
  } else {
    const auto light_sample = light->Sample(inct.pos, sampler);
    if (light_sample.radiance.IsBlack() || light_sample.pdf <= 0.f) return {};

    const float shadow_ray_len =
        (light_sample.pos - inct.pos).length() - Ray::Eps();
    if (shadow_ray_len < Ray::Eps()) return {};

    const Vector3f inct2light = (light_sample.pos - inct.pos).normalized();
    const Ray shadow_ray{inct.pos, inct2light, Ray::Eps(),
                         0.99f * shadow_ray_len};
    if (scene->Occlude(shadow_ray)) return {};

    auto med = inct.GetMedium(inct2light);
    const auto bsdf_f =
        sp.bsdf->EvalAll(inct2light, inct.wr, TransMode::Radiance);
    if (bsdf_f.IsBlack()) return {};

    const Spectrum f = med->Tr(light_sample.pos, inct.pos, sampler) *
                       light_sample.radiance * bsdf_f *
                       std::abs(dot(inct2light, inct.geometry_normal));
    const float bsdf_pdf = sp.bsdf->PdfAll(inct2light, inct.wr);

    return f / light_sample.pdf * PowerHeuristic(light_sample.pdf, bsdf_pdf);
  }
}

inline Spectrum MISSampleBsdf(const Scene* scene,
                              const PrimitiveIntersection& inct,
                              const ShadingPoint& sp, Sampler* sampler) {
  auto bsdf_sample =
      sp.bsdf->SampleAll(inct.wr, TransMode::Radiance, sampler->Next3D());
  if (bsdf_sample.f.IsBlack() || bsdf_sample.pdf == 0.f) return {};

  bsdf_sample.dir = bsdf_sample.dir.normalized();

  const Ray ray{inct.EpsOffset(bsdf_sample.dir), bsdf_sample.dir};
  PrimitiveIntersection pri_inct;
  bool intersected = scene->Intersect(ray, &pri_inct);

  const Medium* medium = inct.GetMedium(bsdf_sample.dir);

  if (!intersected) {
    Spectrum envir{};

    // TODO: env light
    // if (auto light = scene->EnvirLight())

    return envir;
  }

  auto light = pri_inct.primitive->AsLight();
  if (!light) return {};

  const auto light_radiance = light->Radiance(
      pri_inct.pos, pri_inct.geometry_normal, pri_inct.uv, pri_inct.wr);
  if (light_radiance.IsBlack()) return {};

  const auto tr = medium->Tr(ray.o, pri_inct.pos, sampler);
  const auto f = tr * light_radiance * bsdf_sample.f *
                 std::abs(dot(inct.geometry_normal, ray.d));

  if (bsdf_sample.is_delta) return f / bsdf_sample.pdf;

  const float light_pdf =
      light->Pdf(ray.o, pri_inct.pos, pri_inct.geometry_normal);
  return f / bsdf_sample.pdf * PowerHeuristic(bsdf_sample.pdf, light_pdf);
}

AJ_END
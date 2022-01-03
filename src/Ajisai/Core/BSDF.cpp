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

#include "Ajisai/Core/BSDF.h"

#include "Ajisai/Core/Mesh.h"
#include "Ajisai/Core/Warp.h"

namespace Ajisai::Core {

float FrDielectric(float cosThetaI, float etaI, float etaT) {
  cosThetaI = std::clamp(cosThetaI, -1.f, 1.f);
  bool entering = cosThetaI > 0.f;
  if (!entering) {
    std::swap(etaI, etaT);
    cosThetaI = std::abs(cosThetaI);
  }
  float sinThetaI = std::sqrt(std::max(0.f, 1 - cosThetaI * cosThetaI));
  float sinThetaT = etaI / etaT * sinThetaI;
  if (sinThetaT >= 1) return 1.f;
  float cosThetaT = std::sqrt(std::max(0.f, 1 - sinThetaT * sinThetaT));

  float Rparl = ((etaT * cosThetaI) - (etaI * cosThetaT)) /
                ((etaT * cosThetaI) + (etaI * cosThetaT));
  float Rperp = ((etaI * cosThetaI) - (etaT * cosThetaT)) /
                ((etaI * cosThetaI) + (etaT * cosThetaT));
  return (Rparl * Rparl + Rperp * Rperp) / 2;
}

// https://seblagarde.wordpress.com/2013/04/29/memo-on-fresnel-equations/
Math::Spectrum FrConductor(float cosThetaI, const Math::Spectrum& etai,
                           const Math::Spectrum& etat,
                           const Math::Spectrum& k) {
  using Math::Spectrum;
  cosThetaI = std::clamp(cosThetaI, -1.f, 1.f);
  Spectrum eta = etat / etai;
  Spectrum etak = k / etai;

  float cosThetaI2 = cosThetaI * cosThetaI;
  float sinThetaI2 = 1. - cosThetaI2;
  Spectrum eta2 = eta * eta;
  Spectrum etak2 = etak * etak;

  Spectrum t0 = eta2 - etak2 - Spectrum{sinThetaI2};
  Spectrum a2plusb2 = Math::sqrt(t0 * t0 + 4 * eta2 * etak2);
  Spectrum t1 = a2plusb2 + Spectrum{cosThetaI2};
  Spectrum a = Math::sqrt(0.5f * (a2plusb2 + t0));
  Spectrum t2 = (float)2 * cosThetaI * a;
  Spectrum Rs = (t1 - t2) / (t1 + t2);

  Spectrum t3 = cosThetaI2 * a2plusb2 + Spectrum{sinThetaI2 * sinThetaI2};
  Spectrum t4 = t2 * sinThetaI2;
  Spectrum Rp = Rs * (t3 - t4) / (t3 + t4);

  return 0.5 * (Rp + Rs);
}

inline float cosTheta(const Math::Vector3f& w) { return w.z(); }
inline float absCosTheta(const Math::Vector3f& w) {
  return std::abs(cosTheta(w));
}
inline Math::Vector3f reflect(const Math::Vector3f& w,
                              const Math::Vector3f& n) {
  return -1.0f * w + 2.0f * Math::dot(w, n) * n;
}

inline bool Refract(const Math::Vector3f& wi, const Math::Vector3f& n,
                    float eta, Math::Vector3f* wt) {
  // Compute $\cos \theta_\roman{t}$ using Snell's law
  float cosThetaI = Math::dot(n, wi);
  float sin2ThetaI = std::max(float(0), float(1 - cosThetaI * cosThetaI));
  float sin2ThetaT = eta * eta * sin2ThetaI;

  // Handle total internal reflection for transmission
  if (sin2ThetaT >= 1) return false;
  float cosThetaT = std::sqrt(1 - sin2ThetaT);
  *wt = eta * -wi + (eta * cosThetaI - cosThetaT) * n;
  return true;
}

void BSDF::Sample(BSDFSamplingRecord& rec) const {
  bxdfs[0]->Sample(rec);
  rec.type = bxdfs[0]->GetType();
  // rec.wi = squareToCosineHemisphere(rec.u);
  // if (rec.wo.z() * rec.wi.z() < 0) {
  //   rec.wi.z() *= -1;
  // }
  // rec.pdf = absCosTheta(rec.wi) / Math::Constants<float>::pi();
  // rec.f = R / Math::Constants<float>::pi();
}

Math::Spectrum BSDF::Evaluate(const Math::Vector3f& wo,
                              const Math::Vector3f& wi) const {
  // if (wo.z() * wo.z() >= 0) {
  //   auto f = R / Math::Constants<float>::pi();
  //   return f;
  // }
  // return Math::Spectrum(0);
  return bxdfs[0]->Evaluate(wo, wi);
}

float BSDF::EvaluatePdf(const Math::Vector3f& wo,
                        const Math::Vector3f& wi) const {
  // return absCosTheta(wi) / Math::Constants<float>::pi();
  return bxdfs[0]->EvaluatePdf(wo, wi);
}

Fresnel::~Fresnel() {}

Math::Spectrum FresnelConductor::Evaluate(float cosThetaI) const {
  return FrConductor(std::abs(cosThetaI), etaI, etaT, k);
}

Math::Spectrum FresnelDielectric::Evaluate(float cosThetaI) const {
  return Math::Spectrum{FrDielectric(cosThetaI, etaI, etaT)};
}

void LambertianReflection::Sample(BSDFSamplingRecord& rec) const {
  rec.wi = squareToCosineHemisphere(rec.u);
  if (rec.wo.z() * rec.wi.z() < 0) {
    rec.wi.z() *= -1;
  }
  rec.pdf = absCosTheta(rec.wi) / Math::Constants<float>::pi();
  rec.f = R / Math::Constants<float>::pi();
}

Math::Spectrum LambertianReflection::Evaluate(const Math::Vector3f& wo,
                                              const Math::Vector3f& wi) const {
  if (wo.z() * wo.z() >= 0) {
    auto f = R / Math::Constants<float>::pi();
    return f;
  }
  return Math::Spectrum(0);
}

float LambertianReflection::EvaluatePdf(const Math::Vector3f& wo,
                                        const Math::Vector3f& wi) const {
  return absCosTheta(wi) / Math::Constants<float>::pi();
}

void SpecularReflection::Sample(BSDFSamplingRecord& rec) const {
  rec.wi = Math::Vector3f{-rec.wo.x(), -rec.wo.y(), rec.wo.z()};

  rec.pdf = 1;
  rec.f = fresnel->Evaluate(cosTheta(rec.wi)) * R / absCosTheta(rec.wi);
}

Math::Spectrum SpecularReflection::Evaluate(const Math::Vector3f& wo,
                                            const Math::Vector3f& wi) const {
  return Math::Spectrum(0);
}

float SpecularReflection::EvaluatePdf(const Math::Vector3f& wo,
                                      const Math::Vector3f& wi) const {
  return 0.f;
}

void SpecularTransmission::Sample(BSDFSamplingRecord& rec) const {
  bool entering = cosTheta(rec.wo) > 0;
  float etaI = entering ? etaA : etaB;
  float etaT = entering ? etaB : etaA;

  if (!Refract(rec.wo, Faceforward(Math::Vector3f{0.f, 0.f, 1.f}, rec.wo),
               etaI / etaT, &rec.wi))
    return;

  rec.pdf = 1;
  Math::Spectrum ft =
      T * (Math::Spectrum{1.f} - fresnel.Evaluate(cosTheta(rec.wi)));
  if (mode == TransportMode::eRadiance) ft *= (etaI * etaI) / (etaT * etaT);
  rec.f = ft / absCosTheta(rec.wi);
}

Math::Spectrum SpecularTransmission::Evaluate(const Math::Vector3f& wo,
                                              const Math::Vector3f& wi) const {
  return Math::Spectrum(0.f);
}

float SpecularTransmission::EvaluatePdf(const Math::Vector3f& wo,
                                        const Math::Vector3f& wi) const {
  return 0.f;
}

void FresnelSpecular::Sample(BSDFSamplingRecord& rec) const {
  float F = FrDielectric(cosTheta(rec.wo), etaA, etaB);
  if (rec.u[0] < F) {
    rec.wi = Math::Vector3f{-rec.wo.x(), -rec.wo.y(), rec.wo.z()};
    rec.type = BxDFType(BSDF_SPECULAR | BSDF_REFLECTION);
    rec.pdf = F;
    rec.f = F * R / absCosTheta(rec.wi);
  } else {
    bool entering = cosTheta(rec.wo) > 0;
    float etaI = entering ? etaA : etaB;
    float etaT = entering ? etaB : etaA;

    if (!Refract(rec.wo, Faceforward(Math::Vector3f{0.f, 0.f, 1.f}, rec.wo),
                 etaI / etaT, &rec.wi))
      return;

    Math::Spectrum ft = T * (1 - F);
    if (mode == TransportMode::eRadiance) ft *= (etaI * etaI) / (etaT * etaT);
    rec.type = BxDFType(BSDF_SPECULAR | BSDF_TRANSMISSION);
    rec.pdf = 1 - F;
    rec.f = ft / absCosTheta(rec.wi);
  }
}

Math::Spectrum FresnelSpecular::Evaluate(const Math::Vector3f& wo,
                                         const Math::Vector3f& wi) const {
  return Math::Spectrum(0.f);
}

float FresnelSpecular::EvaluatePdf(const Math::Vector3f& wo,
                                   const Math::Vector3f& wi) const {
  return 0.f;
}

}  // namespace Ajisai::Core
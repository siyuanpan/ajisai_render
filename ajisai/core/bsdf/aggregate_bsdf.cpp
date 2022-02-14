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
#include <ajisai/core/bsdf/aggregate_bsdf.h>
#include <ajisai/utility/distribution.h>
#include <numeric>

AJ_BEGIN

AggregateBSDF::AggregateBSDF(const Vector3f& ng, const Vector3f& ns,
                             const Spectrum& albedo) noexcept
    : BSDF(ng, ns), albedo_{albedo} {}

void AggregateBSDF::AddComponent(float weight, Rc<BSDFComponent> component) {
  weights_.push_back(weight);
  components_.push_back(component);
}

BSDFSampleResult AggregateBSDF::SampleAll(const Vector3f& wo, TransMode mode,
                                          const Vector3f& sam) const noexcept {
  // process black fringes
  if (CauseBlackFringes(wo)) {
    return SampleBlackFringes(wo, mode, sam);
  }

  const Vector3f lwo = shading_coord_.World2Local(wo).normalized();
  if (lwo.z() <= 0.f) return kBSDFSampleResultInvalid;

  float weight_sum = std::accumulate(weights_.begin(), weights_.end(), 0.f);

  if (weight_sum <= 0.f) return kBSDFSampleResultInvalid;

  Distribution comp_selector{weights_.data(), weights_.size()};

  auto idx = comp_selector.SampleDiscrete(sam.x());

  const BSDFComponent* sam_comp = components_[idx].get();
  float weight = weights_[idx];

  if (!sam_comp) return kBSDFSampleResultInvalid;

  auto sam_ret = sam_comp->Sample(lwo, mode, Vector2f{sam.y(), sam.z()});
  if (!sam_ret.Valid()) {
    return kBSDFSampleResultInvalid;
  }

  sam_ret.lwi = sam_ret.lwi.normalized();
  sam_ret.pdf *= weight;

  for (int i = 0; i < weights_.size(); ++i) {
    if (i != idx) {
      sam_ret.f += components_[i]->Eval(sam_ret.lwi, lwo, mode);
      sam_ret.pdf += components_[i]->Pdf(sam_ret.lwi, lwo) * weights_[i];
    }
  }

  const Vector3f wi = shading_coord_.Local2World(sam_ret.lwi);
  const float factor =
      CorrectShadingNormal(geometry_normal_, shading_normal_, wi);
  //   printf("(%f %f %f) (%f %f %f)\n", sam_ret.lwi[0], sam_ret.lwi[1],
  //          sam_ret.lwi[2], wi[0], wi[1], wi[2]);

  return BSDFSampleResult{wi, sam_ret.f * factor, sam_ret.pdf, false};
}

Spectrum AggregateBSDF::EvalAll(const Vector3f& wi, const Vector3f& wo,
                                TransMode mode) const noexcept {
  if (CauseBlackFringes(wo)) {
    return EvalBlackFringes(wi, wo, mode);
  }

  const Vector3f& lwi = shading_coord_.World2Local(wi).normalized();
  const Vector3f& lwo = shading_coord_.World2Local(wo).normalized();

  // if (lwi.z() == 0.f || lwo.z() == 0.f) return {};
  if (lwi.z() <= 0.f || lwo.z() <= 0.f) return {};

  Spectrum ret{};
  for (auto i = 0; i < weights_.size(); ++i) {
    ret += components_[i]->Eval(lwi, lwo, mode);
  }

  return ret;
}

float AggregateBSDF::PdfAll(const Vector3f& wi,
                            const Vector3f& wo) const noexcept {
  if (CauseBlackFringes(wo)) {
    return PdfBlackFringes(wi, wo);
  }

  const Vector3f& lwi = shading_coord_.World2Local(wi).normalized();
  const Vector3f& lwo = shading_coord_.World2Local(wo).normalized();

  // if (lwi.z() == 0.f || lwo.z() == 0.f) return {};
  if (lwi.z() <= 0.f || lwo.z() <= 0.f) return {};

  float weight_sum = 0;
  float pdf = 0;

  for (int i = 0; i < weights_.size(); ++i) {
    weight_sum += weights_[i];
    pdf += weights_[i] * components_[i]->Pdf(lwi, lwo);
  }

  return weight_sum > 0 ? pdf / weight_sum : 0;
}

BSDFSampleResult AggregateBSDF::Sample(const Vector3f& wo, TransMode mode,
                                       const Vector3f& sam,
                                       uint8_t type) const noexcept {
  // process black fringes
  if (CauseBlackFringes(wo)) {
    return SampleBlackFringes(wo, mode, sam);
  }

  const Vector3f lwo = shading_coord_.World2Local(wo).normalized();
  if (lwo.z() <= 0.f) return kBSDFSampleResultInvalid;

  float weight_sum = 0.f;
  for (size_t i = 0; i < weights_.size(); ++i) {
    const auto comp = components_[i];
    if (comp->Contained(type)) {
      weight_sum += weights_[i];
    }
  }

  if (weight_sum <= 0.f) return kBSDFSampleResultInvalid;

  float comp_selector = (sam.x() - 0.001f) * weight_sum;
  const BSDFComponent* sam_comp = nullptr;
  float weight = 0.f;
  for (size_t i = 0; i < weights_.size(); ++i) {
    const auto comp = components_[i];
    if (!comp->Contained(type)) continue;

    if (comp_selector <= weights_[i]) {
      sam_comp = comp.get();
      weight = weights_[i];
      break;
    }
    comp_selector -= weights_[i];
  }

  if (!sam_comp) return kBSDFSampleResultInvalid;

  auto sam_ret = sam_comp->Sample(lwo, mode, Vector2f{sam.y(), sam.z()});
  if (!sam_ret.Valid()) {
    return kBSDFSampleResultInvalid;
  }

  sam_ret.lwi = sam_ret.lwi.normalized();
  sam_ret.pdf *= weight;

  for (int i = 0; i < weights_.size(); ++i) {
    const auto comp = components_[i];
    if (comp.get() == sam_comp || !comp->Contained(type)) continue;

    sam_ret.f += components_[i]->Eval(sam_ret.lwi, lwo, mode);
    sam_ret.pdf += components_[i]->Pdf(sam_ret.lwi, lwo) * weights_[i];
  }

  const Vector3f wi = shading_coord_.Local2World(sam_ret.lwi);
  const float factor =
      CorrectShadingNormal(geometry_normal_, shading_normal_, wi);
  //   printf("(%f %f %f) (%f %f %f)\n", sam_ret.lwi[0], sam_ret.lwi[1],
  //          sam_ret.lwi[2], wi[0], wi[1], wi[2]);

  return BSDFSampleResult{wi, sam_ret.f * factor, sam_ret.pdf, false};
}

Spectrum AggregateBSDF::Eval(const Vector3f& wi, const Vector3f& wo,
                             TransMode mode, uint8_t type) const noexcept {
  if (CauseBlackFringes(wo)) {
    return EvalBlackFringes(wi, wo, mode);
  }

  const Vector3f& lwi = shading_coord_.World2Local(wi).normalized();
  const Vector3f& lwo = shading_coord_.World2Local(wo).normalized();

  // if (lwi.z() == 0.f || lwo.z() == 0.f) return {};
  if (lwi.z() <= 0.f || lwo.z() <= 0.f) return {};

  Spectrum ret{};
  for (auto i = 0; i < weights_.size(); ++i) {
    const auto comp = components_[i];
    if (comp->Contained(type)) ret += components_[i]->Eval(lwi, lwo, mode);
  }

  return ret;
}

float AggregateBSDF::Pdf(const Vector3f& wi, const Vector3f& wo,
                         uint8_t type) const noexcept {
  if (CauseBlackFringes(wo)) {
    return PdfBlackFringes(wi, wo);
  }

  const Vector3f& lwi = shading_coord_.World2Local(wi).normalized();
  const Vector3f& lwo = shading_coord_.World2Local(wo).normalized();

  // if (lwi.z() == 0.f || lwo.z() == 0.f) return {};
  if (lwi.z() <= 0.f || lwo.z() <= 0.f) return {};

  float weight_sum = 0;
  float pdf = 0;

  for (int i = 0; i < weights_.size(); ++i) {
    const auto comp = components_[i];
    if (!comp->Contained(type)) continue;

    weight_sum += weights_[i];
    pdf += weights_[i] * components_[i]->Pdf(lwi, lwo);
  }

  return weight_sum > 0 ? pdf / weight_sum : 0;
}

AJ_END
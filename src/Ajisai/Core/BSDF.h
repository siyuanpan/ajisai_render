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

#ifndef AJISAI_CORE_BSDF_H_
#define AJISAI_CORE_BSDF_H_

#include <cassert>
#include "Ajisai/Core/CoordinateSystem.h"
#include "Ajisai/Core/Sampler.h"
#include "Ajisai/Math/Math.h"
#include "Ajisai/Math/Spectrum.hpp"

namespace Ajisai::Core {

// struct BSDFSamplingRecord {
//   const Math::Vector3f wo;
//   inline BSDFSamplingRecord(const ScatteringEvent& event, Sampler* sampler);
// };

struct BSDFSamplingRecord;
class BxDF;

enum BSDFType : uint32_t {
  BSDF_NONE = 0u,
  BSDF_DIFFUSE = 1u << 0u,
  BSDF_SPECULAR = 1u << 1u
};

class BSDF {
  static constexpr int MAXBxDFs = 4;
  // Math::Spectrum R;
  Math::Vector3f Ng;
  const CoordinateSystem frame;
  std::shared_ptr<BxDF> bxdfs[MAXBxDFs];
  int nBxDF = 0;

 public:
  BSDF(const Math::Vector3f& Ng, const Math::Vector3f& Ns)
      : Ng(Ng), frame(Ns) {}

  auto toWorld(const Math::Vector3f& w) const { return frame.toWorld(w); }
  auto toLocal(const Math::Vector3f& w) const { return frame.toLocal(w); }

  void add(const std::shared_ptr<BxDF>& bsdf) {
    assert(nBxDF < MAXBxDFs && "too many BxDf");
    bxdfs[nBxDF++] = std::move(bsdf);
  }

  void Sample(BSDFSamplingRecord& rec) const;
  Math::Spectrum Evaluate(const Math::Vector3f&, const Math::Vector3f&) const;
  float EvaluatePdf(const Math::Vector3f&, const Math::Vector3f&) const;
};

class BxDF {
 public:
  BxDF() {}

  virtual BSDFType GetType() const = 0;
  virtual void Sample(BSDFSamplingRecord& rec) const = 0;
  virtual Math::Spectrum Evaluate(const Math::Vector3f&,
                                  const Math::Vector3f&) const = 0;
  virtual float EvaluatePdf(const Math::Vector3f&,
                            const Math::Vector3f&) const = 0;
};

class LambertianReflection : public BxDF {
 public:
  LambertianReflection(const Math::Spectrum& R) : R(R) {}

  virtual BSDFType GetType() const { return BSDFType::BSDF_DIFFUSE; }
  virtual void Sample(BSDFSamplingRecord& rec) const;
  virtual Math::Spectrum Evaluate(const Math::Vector3f&,
                                  const Math::Vector3f&) const;
  virtual float EvaluatePdf(const Math::Vector3f&, const Math::Vector3f&) const;

 private:
  Math::Spectrum R;
};

class SpecularReflection : public BxDF {
 public:
  SpecularReflection(const Math::Spectrum& R) : R(R) {}

  virtual BSDFType GetType() const { return BSDFType::BSDF_SPECULAR; }
  virtual void Sample(BSDFSamplingRecord& rec) const;
  virtual Math::Spectrum Evaluate(const Math::Vector3f&,
                                  const Math::Vector3f&) const;
  virtual float EvaluatePdf(const Math::Vector3f&, const Math::Vector3f&) const;

 private:
  Math::Spectrum R;
};

}  // namespace Ajisai::Core

#endif
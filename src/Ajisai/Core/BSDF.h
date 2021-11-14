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

class BSDF {
  Math::Spectrum R;
  Math::Vector3f Ng;
  const CoordinateSystem frame;

 public:
  BSDF(const Math::Spectrum& R, const Math::Vector3f& Ng,
       const Math::Vector3f& Ns)
      : R(R), Ng(Ng), frame(Ns) {}

  auto toWorld(const Math::Vector3f& w) const { return frame.toWorld(w); }
  auto toLocal(const Math::Vector3f& w) const { return frame.toLocal(w); }

  void Sample(BSDFSamplingRecord& rec) const;
  Math::Spectrum Evaluate(const Math::Vector3f&, const Math::Vector3f&) const;
};

// inline BSDFSamplingRecord::BSDFSamplingRecord(const ScatteringEvent& event,
//                                               Sampler* sampler)
//     : wo(event.bsdf->toLocal(event.wo)) {}

}  // namespace Ajisai::Core

#endif
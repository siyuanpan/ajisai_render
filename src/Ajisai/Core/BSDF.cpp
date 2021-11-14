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

inline float cosTheta(const Math::Vector3f& w) { return w.z(); }
inline float absCosTheta(const Math::Vector3f& w) {
  return std::abs(cosTheta(w));
}

void BSDF::Sample(BSDFSamplingRecord& rec) const {
  rec.wi = squareToCosineHemisphere(rec.u);
  if (rec.wo.z() * rec.wi.z() < 0) {
    rec.wi.z() *= -1;
  }
  rec.pdf = absCosTheta(rec.wi) / Math::Constants<float>::pi();
  rec.f = R / Math::Constants<float>::pi();
}

Math::Spectrum BSDF::Evaluate(const Math::Vector3f& wo,
                              const Math::Vector3f& wi) const {
  if (wo.z() * wo.z() >= 0) {
    auto f = R / Math::Constants<float>::pi();
    return f;
  }
  return Math::Spectrum(0);
}
}  // namespace Ajisai::Core
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
#include <ajisai/core/intersection.h>
#include <ajisai/core/bsdf/bsdf.h>

AJ_BEGIN

class Sampler;

struct SampleOutScatteringResult {
  MediumScattering scattering_point;
  Spectrum throughput;

  const BSDF* phase_function;

  SampleOutScatteringResult(const MediumScattering sp,
                            const Spectrum& throughput,
                            const BSDF* phase_function)
      : scattering_point(sp),
        throughput(throughput),
        phase_function(phase_function) {}

  bool ScatteringHappened() const { return phase_function != nullptr; }
};

class Medium {
 public:
  virtual ~Medium() = default;

  virtual int GetMaxScatteringCount() const noexcept = 0;

  virtual SampleOutScatteringResult SampleScattering(
      const Vector3f& a, const Vector3f& b, Sampler* sampler,
      bool indirect_scattering) const = 0;

  virtual Spectrum Absorbtion(const Vector3f& a, const Vector3f& b,
                              Sampler* sampler) const noexcept = 0;
};

struct MediumInterface {
  Rc<const Medium> in;
  Rc<const Medium> out;
};

AJISAI_API Rc<Medium> CreateVoidMedium();

AJ_END
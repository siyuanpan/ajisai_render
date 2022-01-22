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

#include <algorithm>
#include <cstdio>
#include <vector>
#include <cassert>

AJ_BEGIN

class Distribution {
 public:
  Distribution() = default;

  Distribution(const float* f, size_t n) { SetFunction(f, n); }

  void SetFunction(const float* f, size_t n) {
    func.assign(f, f + n);
    cdf.resize(n + 1);

    float invSize = 1.f / float(n);
    cdf[0] = 0.f;
    for (auto i = 1; i < cdf.size(); ++i) {
      cdf[i] = cdf[i - 1] + func[i - 1] * invSize;
    }

    funcInt = cdf.back();
    if (funcInt > 0.f) {
      for (auto& c : cdf) {
        c /= funcInt;
      }
    } else {
      for (int i = 0; i < n + 1; ++i) {
        cdf[i] = float(i) / float(n);
      }
    }
  }

  int Count() const { return (int)func.size(); }

  float SampleContinuous(float u, float* pdf, int* off = nullptr) const {
    auto upper = std::upper_bound(cdf.begin(), cdf.end(), u,
                                  [](float f1, float f2) { return f1 <= f2; });
    auto offset =
        std::clamp((int)std::distance(cdf.begin(), upper) - 1, 0, Count() - 1);

    if (off) *off = offset;

    float du = u - cdf[offset];
    if ((cdf[offset + 1] - cdf[offset]) > 0) {
      assert(cdf[offset + 1] > cdf[offset]);
      du /= (cdf[offset + 1] - cdf[offset]);
    }

    assert(!std::isnan(du));

    if (pdf) *pdf = (funcInt > 0) ? func[offset] / funcInt : 0;

    return (offset + du) / Count();
  }

  int SampleDiscrete(float u, float* pdf = nullptr,
                     float* uRemapped = nullptr) const {
    auto upper = std::upper_bound(cdf.begin(), cdf.end(), u,
                                  [](float f1, float f2) { return f1 <= f2; });
    auto offset =
        std::clamp((int)std::distance(cdf.begin(), upper) - 1, 0, Count() - 1);
    if (pdf) *pdf = (funcInt > 0) ? func[offset] / (funcInt * Count()) : 0.f;
    if (uRemapped)
      *uRemapped = (u - cdf[offset]) / (cdf[offset + 1] - cdf[offset]);
    if (uRemapped) assert(*uRemapped >= 0.f && *uRemapped <= 1.f);
    return offset;
  }

  int DiscretePDF(int index) const {
    assert(index >= 0 & index < Count());
    return func[index] / (funcInt * Count());
  }

  //  private:
  std::vector<float> func, cdf;
  float funcInt;
};

AJ_END
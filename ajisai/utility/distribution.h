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
#include <numeric>
#include <queue>

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

class Distribution2D {
 public:
  Distribution2D() = default;

  Distribution2D(const float* f, size_t size_x, size_t size_y) {
    for (size_t i = 0; i < size_y; ++i) {
      conditional_.emplace_back(BoxNew<Distribution>(&f[i * size_x], size_x));
    }

    auto marginal_f = BoxNew<float[]>(size_y);
    for (size_t i = 0; i < size_y; ++i) {
      marginal_f[i] = conditional_[i]->funcInt;
    }

    marginal_ = BoxNew<Distribution>(marginal_f.get(), size_y);
  }

  void SampleContinuous(float u, float v, float* sample_u, float* sample_v,
                        float* pdf) const {
    float pdfs[2];
    int iv;
    *sample_v = marginal_->SampleContinuous(v, &pdfs[1], &iv);
    *sample_u = conditional_[iv]->SampleContinuous(u, &pdfs[0]);
    *pdf = pdfs[0] * pdfs[1];
    assert(!std::isnan(*pdf));
  }

  float Pdf(float u, float v) const {
    int iu = std::clamp((int)(u * conditional_[0]->Count()), 0,
                        conditional_[0]->Count() - 1);
    int iv =
        std::clamp((int)(v * marginal_->Count()), 0, marginal_->Count() - 1);
    if (conditional_[iv]->funcInt * marginal_->funcInt == 0.0f) return 0.f;

    return (conditional_[iv]->func[iu] * marginal_->func[iv]) /
           (conditional_[iv]->funcInt * marginal_->funcInt);
  }

 private:
  std::vector<Box<Distribution>> conditional_;
  Box<Distribution> marginal_;
};

/// Random-number sampling using the Walker-Vose alias method,
/// as described by Keith Schwarz (2011)
/// [http://www.keithschwarz.com/darts-dice-coins]
template <class ValueType, class SizeType>
class AliasMethod {
  static_assert(std::is_floating_point_v<ValueType> &&
                    std::is_integral_v<SizeType>,
                "AliasMethod : ValueType must floating type and SizeType must "
                "integral type");

 public:
  AliasMethod() = default;

  AliasMethod(const ValueType* f, SizeType n) { Init(f, n); }

  void Init(const ValueType* f, SizeType n) {
    const ValueType sum = std::accumulate(f, f + n, ValueType(0.0));
    const ValueType n_div_sum = ValueType(n) / sum;
    tables.resize(n);
    std::queue<SizeType> small, large;

    for (SizeType i = 0; i != n; ++i) {
      const ValueType p = f[i] * n_div_sum;
      tables[i].first = p;
      tables[i].second = i;

      if (p < ValueType(1)) {
        small.push(i);
      } else {
        large.push(i);
      }
    }

    while (!small.empty() && !large.empty()) {
      auto s = small.front(), l = large.front();
      small.pop();
      large.pop();
      tables[s].second = l;
      tables[l].first -= (ValueType(1) - tables[s].first);

      if (tables[l].first < ValueType(1)) {
        small.push(l);
      } else {
        large.push(l);
      }
    }

    for (auto l : large) {
      tables[l].first = ValueType(1);
    }

    for (auto s : small) {
      tables[s].first = ValueType(1);
    }
  }

  void Clear() { tables.clear(); }

  SizeType Sample(ValueType u) const noexcept {
    assert(ValueType(0) <= u && u <= ValueType(1));

    SizeType i = std::clamp(static_cast<SizeType>(u * tables.size()),
                            SizeType(0), (SizeType)tables.size() - SizeType(1));

    const ValueType s = u * tables.size() - i;

    if (s < tables[i].first)
      return i;
    else
      return tables[i].second;
  }

  SizeType Sample(ValueType u1, ValueType u2) const noexcept {
    assert(ValueType(0) <= u1 && u1 <= ValueType(1));
    assert(ValueType(0) <= u2 && u2 <= ValueType(1));

    SizeType i = std::clamp(static_cast<SizeType>(u1 * tables.size()),
                            SizeType(0), (SizeType)tables.size() - SizeType(1));

    if (u2 < tables[i].first)
      return i;
    else
      return tables[i].second;
  }

 private:
  std::vector<std::pair<ValueType, SizeType>> tables;
};

AJ_END
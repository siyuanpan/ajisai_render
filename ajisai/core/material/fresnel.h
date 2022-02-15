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
#include <ajisai/math/spectrum.h>
#include <ajisai/math/functions.h>

AJ_BEGIN

class Fresnel {
 public:
  virtual ~Fresnel() = default;

  virtual Spectrum Eval(float cos_theta_i) const noexcept = 0;
};

class ConductorFresnel : public Fresnel {
 public:
  ConductorFresnel(const Spectrum& eta_out, const Spectrum& eta_in,
                   const Spectrum& k) noexcept
      : eta_out_(eta_out), eta_in_(eta_in), k_(k) {
    eta_2_ = eta_in_ / eta_out_;
    eta_2_ *= eta_2_;

    eta_k_2_ = k_ / eta_out_;
    eta_k_2_ *= eta_k_2_;
  }

  virtual Spectrum Eval(float cos_theta_i) const noexcept override {
    if (cos_theta_i <= 0.f) return {};

    const float cos2 = cos_theta_i * cos_theta_i;
    const float sin2 = std::max(0.f, 1 - cos2);

    const Spectrum t0 = eta_2_ - eta_k_2_ - Spectrum{sin2};
    const Spectrum a2_plus_b2 = aj::sqrt((t0 * t0 + 4.f * eta_2_ * eta_k_2_));
    const Spectrum t1 = a2_plus_b2 + Spectrum{cos2};
    const Spectrum a = aj::sqrt((0.5f * (a2_plus_b2 + t0)));
    const Spectrum t2 = 2.f * cos_theta_i * a;
    const Spectrum rs = (t1 - t2) / (t1 + t2);

    const Spectrum t3 = cos2 * a2_plus_b2 + Spectrum{sin2 * sin2};
    const Spectrum t4 = t2 * sin2;
    const Spectrum rp = rs * (t3 - t4) / (t3 + t4);

    return 0.5f * (rp + rs);
  }

 private:
  Spectrum eta_out_;
  Spectrum eta_in_;
  Spectrum k_;

  Spectrum eta_2_, eta_k_2_;
};

AJ_END
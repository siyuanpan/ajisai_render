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
#include <ajisai/ajisai.h>
#include <ajisai/core/medium/medium.h>
#include <ajisai/core/sampler/sampler.h>
#include <ajisai/core/medium/phase_function.h>

AJ_BEGIN

class HomogeneousMedium : public Medium {
 public:
  HomogeneousMedium(const Spectrum& sigma_a, const Spectrum& sigma_s, float g,
                    int max_scattering_count) {
    sigma_a_ = sigma_a;
    sigma_s_ = sigma_s;
    sigma_t_ = sigma_a + sigma_s;

    g_ = g;

    max_scattering_count_ = max_scattering_count;
  }

  virtual int GetMaxScatteringCount() const noexcept override {
    return max_scattering_count_;
  }

  virtual SampleOutScatteringResult SampleScattering(const Vector3f& a,
                                                     const Vector3f& b,
                                                     Sampler* sampler,
                                                     bool) const override {
    if (sigma_s_.IsBlack()) return {{}, Tr(a, b, sampler), nullptr};

    const int channel = std::min((int)(sampler->Next1D() * 3), 2);
    const float st = -std::log(1 - sampler->Next1D()) / sigma_t_[channel];

    const float t_max = (a - b).length();
    const bool sample_medium = st < t_max;

    Spectrum tr{};
    for (size_t i = 0; i < 3; ++i) {
      tr[i] = std::exp(-sigma_t_[i] * std::min(st, t_max));
    }
    const Spectrum density = sample_medium ? sigma_t_ * tr : tr;

    float pdf = 0;
    for (size_t i = 0; i < 3; ++i) pdf += density[i];

    pdf /= 3.f;
    pdf = std::max(pdf, std::numeric_limits<float>::epsilon());

    Spectrum throughput = tr / pdf;
    if (sample_medium) throughput *= sigma_s_;

    if (sample_medium) {
      MediumScattering sp;
      const auto t = st / t_max;
      sp.pos = (1 - t) * a + t * b;
      sp.medium = this;
      sp.wr = (a - b) / t_max;

      PhaseFunction* pf = new HenyeyGreensteinPhaseFunction(g_, Albedo());

      return {sp, throughput, pf};
    }

    return {{}, throughput, nullptr};
  }

  virtual Spectrum Tr(const Vector3f& a, const Vector3f& b,
                      Sampler* sampler) const noexcept override {
    const Spectrum exp = -sigma_t_ * (a - b).length();
    return Spectrum{std::exp(exp[0]), std::exp(exp[1]), std::exp(exp[2])};
  }

  virtual Spectrum Absorbtion(const Vector3f& a, const Vector3f& b,
                              Sampler* sampler) const noexcept override {
    const Spectrum exp = -sigma_a_ * (a - b).length();
    return Spectrum{std::exp(exp[0]), std::exp(exp[1]), std::exp(exp[2])};
  }

 private:
  Spectrum Albedo() const {
    return sigma_t_.IsBlack() ? Spectrum{1.f} : sigma_s_ / sigma_t_;
  }

  Spectrum sigma_a_;
  Spectrum sigma_s_;
  Spectrum sigma_t_;
  float g_ = 0;

  int max_scattering_count_;
};

Rc<Medium> CreateHomogeneousMedium(const Spectrum& sigma_a,
                                   const Spectrum& sigma_s, float g,
                                   int max_scattering_count) {
  return RcNew<HomogeneousMedium>(sigma_a, sigma_s, g, max_scattering_count);
}

AJ_END
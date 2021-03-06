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

AJ_BEGIN

class VoidMedium : public Medium {
 public:
  virtual int GetMaxScatteringCount() const noexcept override {
    return std::numeric_limits<int>::max();
  }

  virtual SampleOutScatteringResult SampleScattering(const Vector3f&,
                                                     const Vector3f&, Sampler*,
                                                     bool) const override {
    return {{}, Spectrum{1.f}, nullptr};
  }

  virtual Spectrum Tr(const Vector3f& a, const Vector3f& b,
                      Sampler* sampler) const noexcept override {
    return Spectrum{1.f};
  }

  virtual Spectrum Absorbtion(const Vector3f& a, const Vector3f& b,
                              Sampler* sampler) const noexcept override {
    return Spectrum{1.f};
  }
};

Rc<Medium> CreateVoidMedium() {
  static Rc<VoidMedium> ret = RcNew<VoidMedium>();
  return ret;
}

AJ_END
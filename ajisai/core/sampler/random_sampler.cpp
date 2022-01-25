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
#include <ajisai/core/sampler/sampler.h>
#include <ajisai/core/random.h>

AJ_BEGIN

class RandomSampler : public Sampler {
 public:
  virtual void SetSeed(std::size_t seed) override { rng = PCG32(seed); }
  virtual Rc<Sampler> Copy() const override {
    return RcNew<RandomSampler>(*this);
  }
  virtual float Next1D() override { return rng.NextFloat(); }
  virtual Vector2f Next2D() override {
    return {rng.NextFloat(), rng.NextFloat()};
  }

 private:
  PCG32 rng;
};

Rc<Sampler> CreateRandomSampler() { return RcNew<RandomSampler>(); }

AJ_END
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

#ifndef AJISAI_CORE_SAMPLER_H_
#define AJISAI_CORE_SAMPLER_H_

#include <cstdint>
#include <memory>

#include "Ajisai/Core/Random.h"
#include "Ajisai/Math/Math.h"

namespace Ajisai::Core {
class Sampler {
  PCG32 rng;

 public:
  Sampler() = default;
  void SetSeed(std::size_t seed) { rng = PCG32(seed); }
  std::shared_ptr<Sampler> Copy() const {
    return std::make_shared<Sampler>(*this);
  }
  float Next1D() { return rng.next_float(); }
  Math::Vector2f Next2D() { return {rng.next_float(), rng.next_float()}; }
};
}  // namespace Ajisai::Core

#endif
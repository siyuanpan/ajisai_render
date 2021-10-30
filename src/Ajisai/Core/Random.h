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

#ifndef AJISAI_CORE_RANDOM_H_
#define AJISAI_CORE_RANDOM_H_

#include <cstdint>

namespace Ajisai::Core {
// https://en.wikipedia.org/wiki/Permuted_congruential_generator
class PCG32 {
 public:
  PCG32(uint64_t seed = 0) : state(seed + increment) { next_uint32(); }

  uint32_t rotr32(uint32_t x, unsigned r) { return x >> r | x << (-r & 31); }

  uint32_t next_uint32() {
    uint64_t x = state;
    unsigned rot = (unsigned)(x >> 59);  // 59 = 64 - 5

    state = x * multiplier + increment;
    x ^= x >> 18;                             // 18 = (64 - 27) / 2
    return rotr32((uint32_t)(x >> 27), rot);  // 27 = 32 - 5
  }

  // https://github.com/wjakob/pcg32/blob/master/pcg32.h line 103-112
  float next_float() {
    union {
      uint32_t u;
      float f;
    } x;
    x.u = (next_uint32() >> 9) | 0x3f800000u;
    return x.f - 1.0f;
  }

  // https://github.com/wjakob/pcg32/blob/master/pcg32.h line 121-130
  double next_double() {
    union {
      uint64_t u;
      double d;
    } x;
    x.u = ((uint64_t)next_uint32() << 20) | 0x3ff0000000000000ULL;
    return x.d - 1.0;
  }

 private:
  uint64_t state;
  static uint64_t const multiplier = 6364136223846793005u;
  static uint64_t const increment = 1442695040888963407u;
};
}  // namespace Ajisai::Core

#endif
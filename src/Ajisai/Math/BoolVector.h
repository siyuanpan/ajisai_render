#ifndef AJISAI_MATH_BOOLVECTOR_H_
#define AJISAI_MATH_BOOLVECTOR_H_
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

#include <cstdint>
#include <type_traits>

namespace Ajisai::Math {

template <std::size_t size>
class BoolVector {
  static_assert(size != 0, "BoolVector cannot have zero elements");

 public:
  enum : std::size_t { DataSize = (size - 1) / 8 + 1 };

  constexpr BoolVector() noexcept : _data{} {}

  template <class... T, class U = typename std::enable_if<
                            sizeof...(T) + 1 == DataSize, bool>::type>
  constexpr BoolVector(std::uint8_t first, T... next) noexcept
      : _data{first, std::uint8_t(next)...} {}

  constexpr BoolVector(const BoolVector<size>&) noexcept = default;

  std::uint8_t* data() { return _data; }
  constexpr const std::uint8_t* data() const { return _data; }

  constexpr bool operator[](std::size_t i) const {
    return (_data[i / 8] >> i % 8) & 0x01;
  }

  BoolVector<size>& set(std::size_t i, bool value) {
    value ? _data[i / 8] |= (1 << i % 8) : _data[i / 8] &= ~(1 << i % 8);
    return *this;
  }

  bool all() const;

  BoolVector<size> operator~() const;

  BoolVector<size> operator!() const { return operator~(); }

 private:
  enum : std::uint8_t { FullMask = 0xFF, LastMask = (1 << size % 8) - 1 };

  std::uint8_t _data[(size - 1) / 8 + 1];
};

template <std::size_t size>
inline bool BoolVector<size>::all() const {
  for (std::size_t i = 0; i != (size) / 8; ++i)
    if (_data[i] != FullMask) return false;

  if (size % 8 && (_data[DataSize - 1] & LastMask) != LastMask) return false;

  return true;
}

template <std::size_t size>
inline BoolVector<size> BoolVector<size>::operator~() const {
  BoolVector<size> out;

  for (std::size_t i = 0; i != DataSize; ++i) out._data[i] = ~_data[i];

  return out;
}

}  // namespace Ajisai::Math

#endif
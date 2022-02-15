#pragma once

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

#include <cstddef>
#include <utility>
#include <cstdint>

namespace Ajisai::Util {

class Allocator {
 public:
  virtual ~Allocator() {}

  [[nodiscard]] virtual void* Allocate(size_t _size, size_t _alignment) = 0;

  virtual void Deallocate(void* _block) = 0;

  static Allocator* default_allocator();

  static Allocator* set_defaultAllocator(Allocator* _allocator);
};

template <typename _Ty, typename... Args>
_Ty* New(Args&&... args) {
  void* alloc =
      Allocator::default_allocator()->Allocate(sizeof(_Ty), alignof(_Ty));
  return new (alloc) _Ty(std::forward<Args>(args)...);
}

template <typename _Ty>
void Delete(_Ty* object) {
  if (object) {
    (void)object;
    object->~_Ty();
    Allocator::default_allocator()->Deallocate(object);
  }
}

template <typename _Ty>
inline _Ty Align(_Ty value, size_t alignment) {
  return static_cast<_Ty>(value + (alignment - 1)) & (0 - alignment);
}

template <typename _Ty>
inline _Ty* Align(_Ty* address, size_t alignment) {
  return reinterpret_cast<_Ty*>(
      (reinterpret_cast<uintptr_t>(address) + (alignment - 1)) &
      (0 - alignment));
}

}  // namespace Ajisai::Util
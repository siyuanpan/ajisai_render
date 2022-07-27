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
#include <memory_resource>

AJ_BEGIN

class MemoryArena {
 public:
  explicit MemoryArena(size_t byte_size = 4096) : mbr_{byte_size} {}

  MemoryArena(const MemoryArena& arena) = delete;
  MemoryArena& operator=(const MemoryArena& arena) = delete;

  template <typename T, typename... Args>
  [[nodiscard]] T* Create(Args&&... args) {
    // TODO: support object delete
    // static_assert(std::is_trivially_destructible_v<T> &&
    //               "arena now only support trivially data struct");

    void* mem = mbr_.allocate(sizeof(T), alignof(T));
    T* obj = new (mem) T(std::forward<Args>(args)...);

    return obj;
  }

  void Release() { mbr_.release(); }

 private:
  std::pmr::monotonic_buffer_resource mbr_;
};

AJ_END
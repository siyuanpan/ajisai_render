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

#include "Allocator.h"

#include <atomic>
#include <cassert>
#include <cstdlib>

namespace Ajisai::Util {

namespace {
struct Header {
  void* unaligned;
  size_t size;
};

}  // namespace

class HeapAllocator : public Allocator {
 public:
  HeapAllocator() { allocation_count.store(0); }

  virtual ~HeapAllocator() {
    assert(allocation_count.load() == 0 && "Memory leak detected");
  }

 protected:
  [[nodiscard]] virtual void* Allocate(size_t size, size_t alignment) {
    const size_t allocateSize = size + sizeof(Header) + alignment - 1;
    uint8_t* unaligned = reinterpret_cast<uint8_t*>(malloc(allocateSize));
    if (!unaligned) {
      return nullptr;
    }

    uint8_t* aligned = Align(unaligned + sizeof(Header), alignment);
    assert(aligned + size <= unaligned + allocateSize);

    Header* header = reinterpret_cast<Header*>(aligned - sizeof(Header));
    assert(reinterpret_cast<uint8_t*>(header) >= unaligned);

    header->unaligned = unaligned;
    header->size = size;

    ++allocation_count;
    return aligned;
  }

  virtual void Deallocate(void* block) {
    if (block) {
      Header* header = reinterpret_cast<Header*>(
          reinterpret_cast<uint8_t*>(block) - sizeof(Header));
      free(header->unaligned);
      --allocation_count;
    }
  }

 private:
  std::atomic_int allocation_count;
};

namespace {
HeapAllocator g_heap_allocator;

Allocator* g_default_allocator = &g_heap_allocator;
}  // namespace

Allocator* Allocator::default_allocator() { return g_default_allocator; }

Allocator* set_defaultAllocator(Allocator* _allocator) {
  Allocator* previous = g_default_allocator;
  g_default_allocator = _allocator;
  return previous;
}

}  // namespace Ajisai::Util
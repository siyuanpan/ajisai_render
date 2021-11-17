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

#ifndef AJISAI_UTIL_PTR_H_
#define AJISAI_UTIL_PTR_H_

#include <cstddef>

namespace Ajisai::Util {

template <class T>
class Ptr {
 public:
  Ptr(std::nullptr_t = nullptr) noexcept : pointer{} {}

  explicit Ptr(T* pointer) noexcept : pointer{pointer} {}

  template <class... Args>
  explicit Ptr(Args&&... args) : pointer{new T{std::move(args)...}} {}

  template <class U,
            class = typename std::enable_if<std::is_base_of<T, U>::value>::type>
  Ptr(Ptr<U>&& other) noexcept : pointer{other.release()} {}

  Ptr(const Ptr<T>&) = delete;

  Ptr(Ptr<T>&& other) noexcept : pointer{other.pointer} {
    other.pointer = nullptr;
  }

  Ptr<T>& operator=(const Ptr<T>&) = delete;

  Ptr<T>& operator=(Ptr<T>&& other) noexcept {
    std::swap(pointer, other.pointer);
    return *this;
  }

  bool operator==(std::nullptr_t) const { return !pointer; }

  bool operator!=(std::nullptr_t) const { return pointer; }

  ~Ptr() { delete pointer; }

  explicit operator bool() const { return pointer; }

  T* get() { return pointer; }
  const T* get() const { return pointer; }

  T* operator->() { return pointer; }

  const T* operator->() const { return pointer; }

  T& operator*() { return *pointer; }

  const T& operator*() const { return *pointer; }

  void reset(T* p = nullptr) {
    delete pointer;
    pointer = p;
  }

  template <class... Args>
  T& emplace(Args&&... args) {
    delete pointer;
    pointer = new T{std::move(args)...};

    return *pointer;
  }

  T* release() {
    T* const out = pointer;
    pointer = nullptr;
    return out;
  }

 private:
  T* pointer;
};

template <class T>
bool operator==(std::nullptr_t, const Ptr<T>& b) {
  return !b == nullptr;
}

template <class T>
bool operator!=(std::nullptr_t, const Ptr<T>& b) {
  return b != nullptr;
}

}  // namespace Ajisai::Util

#endif
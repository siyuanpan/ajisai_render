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
#include <cstddef>
#include <type_traits>

AJ_BEGIN

template <class T>
class Ptr {
 public:
  using Type = T;

  Ptr(std::nullptr_t = nullptr) noexcept : pointer_{} {}

  explicit Ptr(T* pointer) noexcept : pointer_{pointer} {}

  template <class... Args>
  explicit Ptr(Args&&... args) : pointer_{new T{std::move(args)...}} {}

  template <class U,
            class = typename std::enable_if<std::is_base_of<T, U>::value>::type>
  Ptr(Ptr<U>&& other) noexcept : pointer_{other.release()} {}

  Ptr(const Ptr<T>&) = delete;

  Ptr(Ptr<T>&& other) noexcept : pointer_{other.pointer_} {
    other.pointer_ = nullptr;
  }

  Ptr<T>& operator=(const Ptr<T>&) = delete;

  Ptr<T>& operator=(Ptr<T>&& other) noexcept {
    std::swap(pointer_, other.pointer_);
    return *this;
  }

  bool operator==(std::nullptr_t) const { return !pointer_; }

  bool operator!=(std::nullptr_t) const { return pointer_; }

  ~Ptr() { delete pointer_; }

  explicit operator bool() const { return pointer_; }

  T* get() { return pointer_; }
  const T* get() const { return pointer_; }

  T* operator->() { return pointer_; }

  const T* operator->() const { return pointer_; }

  T& operator*() { return *pointer_; }

  const T& operator*() const { return *pointer_; }

  void reset(T* p = nullptr) {
    delete pointer_;
    pointer_ = p;
  }

  template <class... Args>
  T& emplace(Args&&... args) {
    delete pointer_;
    pointer_ = new T{std::move(args)...};

    return *pointer_;
  }

  T* release() {
    T* const out = pointer_;
    pointer_ = nullptr;
    return out;
  }

 private:
  T* pointer_;
};

template <class T>
bool operator==(std::nullptr_t, const Ptr<T>& b) {
  return !b == nullptr;
}

template <class T>
bool operator!=(std::nullptr_t, const Ptr<T>& b) {
  return b != nullptr;
}

AJ_END
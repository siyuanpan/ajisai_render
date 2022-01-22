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
class Ref {
 public:
  using Type = T;

  constexpr Ref(T& ref) noexcept : reference_{&ref} {}

  Ref(T&&) = delete;

  template <class U,
            class = typename std::enable_if<std::is_base_of<T, U>::value>::type>
  constexpr Ref(Ref<U> other) noexcept : reference_{other.reference_} {}

  constexpr operator T&() const { return *reference_; }
  constexpr operator Ref<const T>() const { return *reference_; }

  constexpr T& get() const { return *reference_; }

  constexpr T* operator->() const { return reference_; }

  constexpr T& operator*() const { return *reference_; }

 private:
  template <class U>
  friend class Ref;

  T* reference_;
};

AJ_END
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

#include "Unicode.h"

#ifdef AJISAI_TARGET_WINDOWS
#  define WIN32_LEAN_AND_MEAN 1
#  define VC_EXTRALEAN
#  include <windows.h>
#endif

namespace Ajisai::Util {

#if defined(AJISAI_TARGET_WINDOWS)

namespace {
std::wstring widen(const char* const text, const int size) {
  if (!size) return {};
  /* WCtoMB counts the trailing \0 into size, which we have to cut */
  std::wstring result(MultiByteToWideChar(CP_UTF8, 0, text, size, nullptr, 0) -
                          (size == -1 ? 1 : 0),
                      0);
  MultiByteToWideChar(CP_UTF8, 0, text, size, &result[0], result.size());
  return result;
}

std::string narrow(const wchar_t* const text, const int size) {
  if (!size) return {};
  /* WCtoMB counts the trailing \0 into size, which we have to cut */
  std::string result(WideCharToMultiByte(CP_UTF8, 0, text, size, nullptr, 0,
                                         nullptr, nullptr) -
                         (size == -1 ? 1 : 0),
                     0);
  WideCharToMultiByte(CP_UTF8, 0, text, size, &result[0], result.size(),
                      nullptr, nullptr);
  return result;
}
}  // namespace

std::wstring widen(const std::string& text) {
  return widen(text.data(), text.size());
}

std::wstring widen(const char* text) { return widen(text, -1); }

std::string narrow(const std::wstring& text) {
  return narrow(text.data(), text.size());
}

std::string narrow(const wchar_t* text) { return narrow(text, -1); }

#endif

}  // namespace Ajisai::Util
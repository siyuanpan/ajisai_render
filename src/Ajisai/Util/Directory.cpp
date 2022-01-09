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
#include <Ajisai/Ajisai.h>
#include <Ajisai/Util/Directory.h>
#ifdef AJISAI_TARGET_UNIX
#  include <dlfcn.h>
#endif

#ifdef AJISAI_TARGET_WINDOWS
#  include <windows.h>
#endif

#include "Unicode.h"

#include <algorithm>

namespace Ajisai::Util {

std::string fromNativeSeparators(std::string path) {
#ifdef AJISAI_TARGET_WINDOWS
  std::replace(path.begin(), path.end(), '\\', '/');
#endif
  return path;
}

std::string libraryLocation(const void* address) {
#ifdef AJISAI_TARGET_UNIX
  Dl_info info{nullptr, nullptr, nullptr, nullptr};
  if (!dladdr(address, &info)) {
    return {};
  }

  return info.dli_fname;
#elif defined(AJISAI_TARGET_WINDOWS) && !defined(AJISAI_TARGET_WINDOWS_RT)
  HMODULE module{};
  if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                              GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                          reinterpret_cast<const char*>(address), &module)) {
    return {};
  }
  /** @todo get rid of MAX_PATH */
  std::wstring path(MAX_PATH, L'\0');
  std::size_t size = GetModuleFileNameW(module, &path[0], path.size());
  path.resize(size);
  return fromNativeSeparators(narrow(path));
#endif
}

std::string libraryLocation(Impl::FunctionPointer address) {
  return libraryLocation(address.address);
}
}  // namespace Ajisai::Util
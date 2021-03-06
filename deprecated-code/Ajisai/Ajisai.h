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

#ifndef AJISAI_AJISAI_H_
#define AJISAI_AJISAI_H_

#include "Ajisai/configure.h"

#if defined(_WIND32) || defined(_WIN64) || defined(__WINDOWS__)
#  define AJISAI_API_EXPORT __declspec(dllexport)
#  define AJISAI_API_IMPORT __declspec(dllimport)
#elif defined(__linux__)
#  define AJISAI_API_EXPORT __attribute__((visibility("default")))
#  define AJISAI_API_IMPORT __attribute__((visibility("default")))
#elif defined(__APPLE__) && defined(__MACH__)
#  error Mach_OS
#elif defined(unix) || defined(__unix__) || defined(__unix)
#  error Unix_OS
#endif

#define AJISAI_API_STATIC

#ifndef AJISAI_BUILD_STATIC
#  if defined(AJISAI_EXPORTS)
#    define AJISAI_API AJISAI_API_EXPORT
#  else
#    define AJISAI_API AJISAI_API_IMPORT
#  endif
#else
#  define AJISAI_API AJISAI_API_STATIC
#endif

#endif
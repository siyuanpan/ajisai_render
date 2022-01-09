
# Copyright 2021 Siyuan Pan <pansiyuan.cs@gmail.com>

# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

set(UTIL_SRCS Directory.cpp Unicode.cpp)

set(UTIL_HEADERS Directory.h Unicode.h)

add_library(AjisaiUtil SHARED ${UTIL_SRCS} ${UTIL_HEADERS})

target_include_directories(AjisaiUtil PUBLIC ${PROJECT_SOURCE_DIR}/src ${PROJECT_BINARY_DIR}/src)
if(AJISAI_TARGET_UNIX)
    target_link_libraries(AjisaiUtil PRIVATE ${CMAKE_DL_LIBS})
endif()
target_compile_definitions(AjisaiUtil PRIVATE "AJISAI_EXPORTS")

add_library(Ajisai::Util ALIAS AjisaiUtil)
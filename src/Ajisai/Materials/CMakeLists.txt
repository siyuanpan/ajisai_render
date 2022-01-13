
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

set(MATERIAL_SRCS Matte.cpp Mirror.cpp Glass.cpp Ward.cpp)

set(MATERIAL_HEADERS Material.h Matte.h Mirror.h Glass.h Ward.h)

add_library(AjisaiMaterials SHARED ${MATERIAL_SRCS} ${MATERIAL_HEADERS})

target_include_directories(AjisaiMaterials PUBLIC ${PROJECT_SOURCE_DIR}/src ${PROJECT_BINARY_DIR}/src)
target_link_libraries(AjisaiMaterials PUBLIC AjisaiCore)
target_compile_definitions(AjisaiMaterials PRIVATE "AJISAI_EXPORTS")

add_library(Ajisai::Materials ALIAS AjisaiMaterials)
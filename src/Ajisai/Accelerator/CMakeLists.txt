
#
# Copyright 2021 Siyuan Pan <pansiyuan.cs@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#
# option(AJISAI_USE_EMBREE "Use Embree as ray intersection backend" ON)

set(ACCELERATOR_SRCS Accelerator.cpp)

set(ACCELERATOR_HEADERS Accelerator.h)

add_library(AjisaiAccelerator SHARED ${ACCELERATOR_SRCS} ${ACCELERATOR_HEADERS})

target_include_directories(AjisaiAccelerator PUBLIC ${PROJECT_SOURCE_DIR}/src ${PROJECT_BINARY_DIR}/src)
target_link_libraries(AjisaiAccelerator PUBLIC Ajisai::Util Ajisai::PluginManager)

add_library(Ajisai::Accelerator ALIAS AjisaiAccelerator)

add_plugin(BVHAccel BVHAccel.cpp)

target_link_libraries(BVHAccel PRIVATE Ajisai::PluginManager AjisaiAccelerator)

if(AJISAI_USE_EMBREE)
    add_plugin(EmbreeAccel EmbreeAccel.cpp)
    find_package(TBB CONFIG REQUIRED)
    find_package(embree 3 CONFIG REQUIRED)
    target_link_libraries(EmbreeAccel PRIVATE TBB::tbb embree AjisaiAccelerator)
endif()
cmake_minimum_required(VERSION 3.16)

project(Ajisai)

set(CMAKE_CXX_STANDARD 17)

# output directory
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/output/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/output/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/output")

# set output directories for all builds (Debug, Release, etc.)
foreach(OUTPUTCONFIG ${CMAKE_CONFIGURATION_TYPES})
  string(TOUPPER ${OUTPUTCONFIG} OUTPUTCONFIG)
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUTCONFIG}
      ${CMAKE_CURRENT_SOURCE_DIR}/output/lib)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUTCONFIG}
      ${CMAKE_CURRENT_SOURCE_DIR}/output/lib)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUTCONFIG}
      ${CMAKE_CURRENT_SOURCE_DIR}/output)
endforeach(OUTPUTCONFIG CMAKE_CONFIGURATION_TYPES)

find_package(cxxopts CONFIG REQUIRED)
find_package(ryml CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)
find_package(tinyobjloader CONFIG REQUIRED)
find_package(Threads REQUIRED)

file(GLOB AJISAI_CORE_SRC src/Ajisai/Core/*.cpp)
# file(GLOB AJISAI_INTEGRATORS_SRC src/Ajisai/Integrators/*.cpp)
add_library(AjisaiCore SHARED ${AJISAI_CORE_SRC})

# function(add_plugin plugin_name) add_library(${plugin_name} MODULE ${ARGN})
# set_target_properties(${plugin_name} PROPERTIES PREFIX "")
# target_link_libraries(${plugin_name} PRIVATE AjisaiCore)
# target_compile_definitions(${plugin_name} PRIVATE "AJISAI_EXPORTS")
# endfunction()

# add_executable(Ajisai src/Ajisai/App/Ajisai.cpp ${AJISAI_CORE_SRC}
# ${AJISAI_INTEGRATORS_SRC})

find_path(STB_INCLUDE_DIRS "stb.h")
target_include_directories(AjisaiCore PRIVATE ${STB_INCLUDE_DIRS})

target_include_directories(AjisaiCore PUBLIC ${PROJECT_SOURCE_DIR}/src
                                             ${PROJECT_BINARY_DIR}/src)
target_link_libraries(AjisaiCore PUBLIC Threads::Threads)
target_link_libraries(AjisaiCore PRIVATE cxxopts::cxxopts)
# target_link_libraries(AjisaiCore PRIVATE ryml::ryml)
target_link_libraries(AjisaiCore PRIVATE yaml-cpp)
target_link_libraries(AjisaiCore PRIVATE tinyobjloader::tinyobjloader)

# add_subdirectory(src/Ajisai/PluginManager)

option(AJISAI_USE_EMBREE "Use Embree as ray intersection backend" ON)
option(BUILD_STATIC "Build static libraries (default are shared)" OFF)

if(CMAKE_SYSTEM_NAME STREQUAL Emscripten)
  set(AJISAI_TARGET_EMSCRIPTEN 1)
  # It's meaningless to use dynamic libraries with Emscripten
  set(BUILD_STATIC ON)
elseif(UNIX)
  if(APPLE)
    set(AJISAI_TARGET_APPLE 1)
    if(CMAKE_OSX_SYSROOT MATCHES "/iPhoneOS[0-9.]*\\.sdk")
      set(AJISAI_TARGET_IOS 1)
    elseif(CMAKE_OSX_SYSROOT MATCHES "/iPhoneSimulator[0-9.]*\\.sdk")
      set(AJISAI_TARGET_IOS 1)
      set(AJISAI_TARGET_IOS_SIMULATOR 1)
    endif()
  endif()

  if(CMAKE_SYSTEM_NAME STREQUAL Android)
    set(AJISAI_TARGET_ANDROID 1)
    # It's too inconvenient to manually load all shared libs using JNI
    set(BUILD_STATIC ON)
  endif()

  # Emscripten is Unix too, this selects only the other ones
  set(AJISAI_TARGET_UNIX 1)
elseif(WIN32)
  set(AJISAI_TARGET_WINDOWS 1)

  if(WINDOWS_PHONE OR WINDOWS_STORE)
    set(AJISAI_TARGET_WINDOWS_RT 1)
  endif()
endif()

if(BUILD_STATIC)
  set(AJISAI_BUILD_STATIC 1)
endif()

function(add_plugin plugin_name)
  if(AJISAI_TARGET_WINDOWS)
    add_library(${plugin_name} SHARED ${ARGN})
  else()
    add_library(${plugin_name} MODULE ${ARGN})
  endif()
  set_target_properties(${plugin_name} PROPERTIES PREFIX "")
  target_link_libraries(${plugin_name} PRIVATE AjisaiCore)
  target_compile_definitions(${plugin_name} PRIVATE "AJISAI_EXPORTS")
endfunction()

set(AJISAI_INCLUDE_INSTALL_DIR src/Ajisai)

add_subdirectory(src)

add_executable(test src/Ajisai/App/Test.cpp)
target_include_directories(test PUBLIC ${PROJECT_SOURCE_DIR}/src
                                       ${PROJECT_BINARY_DIR}/src)
target_link_libraries(test PRIVATE ryml::ryml)
target_link_libraries(test PRIVATE Ajisai::PluginManager)

add_executable(Ajisai src/Ajisai/App/Ajisai.cpp)
target_link_libraries(Ajisai PUBLIC AjisaiCore Ajisai::PluginManager)
# target_include_directories(Ajisai PUBLIC ${PROJECT_SOURCE_DIR}/src
# ${PROJECT_BINARY_DIR}/src)
target_link_libraries(Ajisai PRIVATE yaml-cpp)
target_link_libraries(Ajisai PRIVATE Ajisai::Util)
target_link_libraries(Ajisai PRIVATE Ajisai::Integrators)
target_link_libraries(Ajisai PRIVATE Ajisai::Accelerator)
target_link_libraries(Ajisai PRIVATE Ajisai::Materials)
if(AJISAI_USE_EMBREE)
  target_compile_definitions(Ajisai PRIVATE "AJISAI_USE_EMBREE")
endif()

# function(add_plugin plugin_name) add_library(${plugin_name} MODULE ${ARGN})
# set_target_properties(${plugin_name} PROPERTIES PREFIX "")
# target_link_libraries(${plugin_name} PRIVATE AjisaiCore) endfunction()

# add_subdirectory(src/Ajisai/Integrators)

cmake_minimum_required(VERSION 3.15)

project(Ajisai)

set(CMAKE_CXX_STANDARD 20)

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
  set(SHARED_OR_STATIC STATIC)
else()
  set(SHARED_OR_STATIC SHARED)
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

set(AJISAI_INCLUDE_INSTALL_DIR ajisai)

add_subdirectory(ajisai)

set(CMAKE_VERBOSE_MAKEFILE ON)

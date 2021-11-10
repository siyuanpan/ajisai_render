cmake_minimum_required(VERSION 3.16)

project(Ajisai)

set(CMAKE_CXX_STANDARD 17)

find_package(cxxopts CONFIG REQUIRED)
find_package(ryml CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)
find_package(tinyobjloader CONFIG REQUIRED)
find_package(Threads REQUIRED)

file(GLOB AJISAI_CORE_SRC src/Ajisai/Core/*.cpp)
file(GLOB AJISAI_INTEGRATORS_SRC src/Ajisai/Integrators/*.cpp)
add_library(AjisaiCore SHARED ${AJISAI_CORE_SRC} ${AJISAI_INTEGRATORS_SRC})

#add_executable(Ajisai src/Ajisai/App/Ajisai.cpp ${AJISAI_CORE_SRC} ${AJISAI_INTEGRATORS_SRC})

find_path(STB_INCLUDE_DIRS "stb.h")
target_include_directories(AjisaiCore PRIVATE ${STB_INCLUDE_DIRS})

target_include_directories(AjisaiCore PUBLIC ${PROJECT_SOURCE_DIR}/src ${PROJECT_BINARY_DIR}/src)
target_link_libraries(AjisaiCore PUBLIC Threads::Threads)
target_link_libraries(AjisaiCore PRIVATE cxxopts::cxxopts)
# target_link_libraries(AjisaiCore PRIVATE ryml::ryml)
target_link_libraries(AjisaiCore PRIVATE yaml-cpp)
target_link_libraries(AjisaiCore PRIVATE tinyobjloader::tinyobjloader)

add_executable(test src/Ajisai/App/Test.cpp)
target_include_directories(test PUBLIC ${PROJECT_SOURCE_DIR}/src ${PROJECT_BINARY_DIR}/src)
target_link_libraries(test PRIVATE ryml::ryml)

add_executable(Ajisai src/Ajisai/App/Ajisai.cpp)
target_link_libraries(Ajisai PUBLIC AjisaiCore)
#target_include_directories(Ajisai PUBLIC ${PROJECT_SOURCE_DIR}/src ${PROJECT_BINARY_DIR}/src)
target_link_libraries(Ajisai PRIVATE yaml-cpp)

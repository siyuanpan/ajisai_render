file(GLOB_RECURSE CORE_SRC "${PROJECT_SOURCE_DIR}/ajisai/core/" *.cpp)
file(GLOB_RECURSE CORE_HEADER "${PROJECT_SOURCE_DIR}/ajisai/core/" *.h)

add_library(AjisaiCore ${SHARED_OR_STATIC} ${CORE_SRC} ${CORE_HEADER})

target_compile_features(AjisaiCore PRIVATE cxx_std_20)
target_include_directories(
  AjisaiCore
  PUBLIC ${PROJECT_SOURCE_DIR} ${PROJECT_BINARY_DIR}
  PRIVATE ${STB_INCLUDE_DIRS})
target_compile_definitions(AjisaiCore PRIVATE "AJISAI_EXPORTS")
target_link_libraries(AjisaiCore PRIVATE TBB::tbb embree)

add_library(ajisai::core ALIAS AjisaiCore)

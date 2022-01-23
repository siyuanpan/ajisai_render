add_executable(ajisai main.cpp)
target_include_directories(ajisai PUBLIC ${PROJECT_SOURCE_DIR}
                                         ${PROJECT_BINARY_DIR})
target_link_libraries(
  ajisai
  PUBLIC ajisai::utility ajisai::factory
  PRIVATE cxxopts::cxxopts)

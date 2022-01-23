file(GLOB UTILITY_SRC "${PROJECT_SOURCE_DIR}/ajisai/utility/*.cpp")
file(GLOB UTILITY_HEADER "${PROJECT_SOURCE_DIR}/ajisai/utility/*.h")

add_library(AjisaiUtility ${SHARED_OR_STATIC} ${UTILITY_SRC} ${UTILITY_HEADER})

target_include_directories(AjisaiUtility PUBLIC ${PROJECT_SOURCE_DIR}
                                                ${PROJECT_BINARY_DIR})
target_compile_definitions(AjisaiUtility PRIVATE "AJISAI_EXPORTS")
target_link_libraries(AjisaiUtility PUBLIC spdlog::spdlog yaml-cpp)

add_library(ajisai::utility ALIAS AjisaiUtility)

file(GLOB_RECURSE FACTORY_SRC "${PROJECT_SOURCE_DIR}/ajisai/factory/" *.cpp)
file(GLOB_RECURSE FACTORY_HEADER "${PROJECT_SOURCE_DIR}/ajisai/factory/" *.h)

add_library(AjisaiFactory ${SHARED_OR_STATIC} ${FACTORY_SRC} ${FACTORY_HEADER})

target_include_directories(AjisaiFactory PUBLIC ${PROJECT_SOURCE_DIR}
                                                ${PROJECT_BINARY_DIR})
target_compile_definitions(AjisaiFactory PRIVATE "AJISAI_EXPORTS")
target_link_libraries(AjisaiFactory PUBLIC AjisaiUtility AjisaiCore yaml-cpp)

add_library(ajisai::factory ALIAS AjisaiFactory)

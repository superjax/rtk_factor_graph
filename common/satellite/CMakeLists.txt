include_directories(../..)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_library(azel azel.cxx)
add_dependencies(azel eigen_ext)
target_link_libraries(azel wgs84)

add_gtest(test_azel SRCS test_azel.cxx)
target_link_libraries(test_azel azel)

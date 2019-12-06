include_directories(../..)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_gtest(test_so3 SRCS test_so3.cxx)
add_dependencies(test_so3 eigen_ext)

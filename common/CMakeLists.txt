add_subdirectory(math)
add_subdirectory(satellite)

include_directories(..)
add_gtest(test_error SRCS test_error.cxx)

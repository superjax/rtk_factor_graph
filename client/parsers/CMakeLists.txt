
include_directories(../..)
add_gtest(test_bit_tools SRCS test_bit_tools.cxx)
add_library(gps gps.cxx)
target_link_libraries(gps utctime)
add_gtest(test_gps SRCS test_gps.cxx DEPS gps)


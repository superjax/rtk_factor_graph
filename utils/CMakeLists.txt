add_library(utctime utctime.cxx)

include_directories(..)
add_gtest(test_utctime SRCS test_utctime.cxx)
target_link_libraries(test_utctime utctime)
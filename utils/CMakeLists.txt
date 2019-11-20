
include_directories(..)

add_library(utctime utctime.cxx)
add_gtest(test_utctime SRCS test_utctime.cxx)
target_link_libraries(test_utctime utctime)

add_library(crc crc.cxx)
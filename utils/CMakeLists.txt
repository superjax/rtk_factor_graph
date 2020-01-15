include_directories(..)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_library(utctime utctime.cxx)
add_gtest(test_utctime SRCS test_utctime.cxx)
target_link_libraries(test_utctime utctime)

add_library(crc crc.cxx)

add_library(wgs84 wgs84.cxx)
add_dependencies(wgs84 eigen_ext)

add_gtest(test_wgs84 SRCS test_wgs84.cxx)
target_link_libraries(test_wgs84 wgs84)

library(crc crc.cxx)

library(wgs84 wgs84.cxx)

add_gtest(test_wgs84 test_wgs84.cxx)
target_link_libraries(test_wgs84 wgs84 print)

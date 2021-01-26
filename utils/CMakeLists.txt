library(crc crc.cxx)

library(wgs84 wgs84.cxx)
library(file file.h)
target_link_libraries(file stdc++fs)

library(progress_bar progress_bar.cxx)

add_gtest(test_wgs84 test_wgs84.cxx)
target_link_libraries(test_wgs84 wgs84 print)

library(split_string split_string.cxx)
library(strip_string strip_string.cxx)

library(config config.cxx)
target_link_libraries(config yaml check)

add_gtest(test_config test_config.cxx)
target_link_libraries(test_config config)

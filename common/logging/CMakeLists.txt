library(header header.cxx)
target_link_libraries(header utctime)
add_gtest(test_header test_header.cxx)
target_link_libraries(test_header header gps galileo glonass)

library(log_reader log_reader.h)
target_link_libraries(log_reader file print stdc++fs)

library(logger logger.cxx)
target_link_libraries(logger file print gps galileo glonass utctime)
add_gtest(test_logger test_logger.cxx)
target_link_libraries(test_logger logger header)

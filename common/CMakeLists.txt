add_subdirectory(math)
add_subdirectory(satellite)
add_subdirectory(ephemeris)

add_gtest(test_error test_error.cxx)
add_gtest(test_logger test_logger.cxx)
add_gtest(test_out test_out.cxx)

library(utctime utctime.cxx)
add_gtest(test_utctime test_utctime.cxx)
target_link_libraries(test_utctime utctime)

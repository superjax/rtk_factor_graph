add_subdirectory(math)
add_subdirectory(satellite)
add_subdirectory(ephemeris)

add_gtest(test_error test_error.cxx)
add_gtest(test_logger test_logger.cxx)
add_gtest(test_out test_out.cxx)

library(utctime utctime.cxx)
add_gtest(test_utctime test_utctime.cxx)
target_link_libraries(test_utctime utctime)

library(print print.cxx)
target_link_libraries(print fmt)
target_include_directories(print PUBLIC ${FMT_INCLUDE_DIRS})
add_gtest(test_print test_print.cxx)
target_link_libraries(test_print print)

library(check check.cxx)
target_link_libraries(check print)

add_gtest(test_hetero_collection test_hetero_collection.cxx)
target_link_libraries(test_hetero_collection states)

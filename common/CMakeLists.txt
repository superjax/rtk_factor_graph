add_subdirectory(math)
add_subdirectory(satellite)
add_subdirectory(logging)
add_subdirectory(ephemeris)

add_gtest(test_error test_error.cxx)
add_gtest(test_out test_out.cxx)


library(utctime utctime.cxx)
add_gtest(test_utctime test_utctime.cxx)
target_link_libraries(test_utctime utctime)

library(quantized_time quantized_time.cxx)
target_link_libraries(quantized_time utctime)
add_gtest(test_quantized_time test_quantized_time.cxx)
target_link_libraries(test_quantized_time quantized_time)

library(print print.cxx)
target_link_libraries(print fmt)
add_gtest(test_print test_print.cxx)
target_link_libraries(test_print print)

library(check check.cxx)
target_link_libraries(check print)

add_gtest(test_hetero_collection test_hetero_collection.cxx)
target_link_libraries(test_hetero_collection states)

add_gtest(test_circular_buffer test_circular_buffer.cxx)
target_link_libraries(test_circular_buffer print)



add_gtest(test_bit_tools test_bit_tools.cxx)

library(gps gps.cxx)
target_link_libraries(gps utctime)
add_gtest(test_gps test_gps.cxx)
target_link_libraries(test_gps gps)
add_benchmark(bench_gps SRCS bench_gps.cxx DEPS gps)

library(glonass glonass.cxx)
target_link_libraries(glonass utctime crc)
add_gtest(test_glonass test_glonass.cxx)
target_link_libraries(test_glonass glonass)
add_benchmark(bench_glonass SRCS bench_glonass.cxx DEPS glonass)

library(galileo galileo.cxx)
target_link_libraries(galileo utctime crc rtklib)
add_gtest(test_galileo test_galileo.cxx)
target_link_libraries(test_galileo galileo)
add_benchmark(bench_galileo SRCS bench_galileo.cxx DEPS galileo)

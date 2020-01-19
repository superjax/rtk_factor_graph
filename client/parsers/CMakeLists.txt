

add_gtest(test_bit_tools SRCS test_bit_tools.cxx)

library(gps gps.cxx)
target_link_libraries(gps utctime)
add_gtest(test_gps SRCS test_gps.cxx DEPS gps)

library(glonass glonass.cxx)
target_link_libraries(glonass utctime crc)
add_dependencies(glonass eigen_ext)
add_gtest(test_glonass SRCS test_glonass.cxx DEPS glonass)

library(galileo galileo.cxx)
target_link_libraries(galileo utctime crc rtklib)
add_gtest(test_galileo SRCS test_galileo.cxx DEPS galileo)

library(azel azel.cxx)
target_link_libraries(azel wgs84)
add_gtest(test_azel test_azel.cxx)
target_link_libraries(test_azel azel)

library(satellite_state satellite_state.cxx)
target_link_libraries(satellite_state glonass utctime)
add_gtest(test_satellite_state test_satellite_state.cxx)
target_link_libraries(test_satellite_state satellite_state rtklib rtklib_adapter)

library(satellite satellite.cxx)
target_link_libraries(satellite satellite_state glonass utctime)
add_gtest(test_satellite test_satellite.cxx)
target_link_libraries(test_satellite satellite)

library(atm_correction atm_correction.cxx)
target_link_libraries(atm_correction satellite_state azel)
add_gtest(test_atm_correction test_atm_correction.cxx)
target_link_libraries(test_atm_correction atm_correction rtklib wgs84 rtklib_adapter)

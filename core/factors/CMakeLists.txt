library(factors factors.cxx)
target_link_libraries(factors states)

add_gtest(test_gnss_factor test_gnss_factor.cxx)
target_link_libraries(test_gnss_factor factors)

add_gtest(test_imu_factor test_imu_factor.cxx)
target_link_libraries(test_imu_factor factors)

add_gtest(test_sw_factor test_sw_factor.cxx)
target_link_libraries(test_sw_factor factors)

add_gtest(test_rel_cp_factor test_rel_cp_factor.cxx)
target_link_libraries(test_rel_cp_factor factors)

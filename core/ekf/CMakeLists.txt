add_subdirectory(ekf_gen)

ekf_cpp(rtk_ekf_base rtk_ekf_base.yaml)

library(rtk_ekf rtk_ekf.cxx)
target_link_libraries(rtk_ekf rtk_ekf_base satellite_cache)

add_gtest(rtk_ekf_test rtk_ekf_test.cxx)
target_link_libraries(rtk_ekf_test rtk_ekf log_writer)

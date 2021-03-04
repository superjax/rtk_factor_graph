add_subdirectory(ekf_gen)

ekf_cpp(rtk_ekf_base rtk_ekf_base.yaml)

library(rtk_ekf rtk_ekf.cxx)
target_link_libraries(rtk_ekf rtk_ekf_base satellite_cache)

add_gtest(rtk_ekf_test rtk_ekf_test.cxx)
target_link_libraries(rtk_ekf_test rtk_ekf log_writer)

library(rtk_ekf_estimator rtk_ekf_estimator.h)
target_link_libraries(
    rtk_ekf_estimator
    rtk_ekf
    sat_manager
    utctime
    satellite_cache
    gtest
)

add_gtest(rtk_ekf_estimator_test rtk_ekf_estimator_test.cxx)
target_link_libraries(rtk_ekf_estimator_test rtk_ekf_estimator)

add_benchmark(
    rtk_ekf_bench
    SRCS rtk_ekf_bench.cxx
    DEPS rtk_ekf
)

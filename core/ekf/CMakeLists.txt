add_subdirectory(ekf_gen)

ekf_cpp(rtk_ekf rtk_ekf.yaml)

library(rtk_ekf_impl rtk_ekf_impl.cxx)
target_link_libraries(rtk_ekf_impl satellite_cache rtk_ekf)

add_gtest(rtk_ekf_test rtk_ekf_test.cxx)
target_link_libraries(
    rtk_ekf_test
    rtk_ekf
    log_writer
    rtk_ekf_impl
    rtk_ekf_state
    rtk_ekf_types
)

library(rtk_ekf_estimator rtk_ekf_estimator.cxx)
target_link_libraries(
    rtk_ekf_estimator
    rtk_ekf
    sat_manager
    utctime
    satellite_cache
    gtest
    yaml
    log_writer
)

add_gtest(rtk_ekf_estimator_test rtk_ekf_estimator_test.cxx)
target_link_libraries(rtk_ekf_estimator_test rtk_ekf_estimator)

add_benchmark(
    rtk_ekf_bench
    SRCS rtk_ekf_bench.cxx
    DEPS rtk_ekf satellite_cache rtk_ekf_impl
)

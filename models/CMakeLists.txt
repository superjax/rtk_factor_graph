library(imu_state imu_state.cxx)
add_gtest(test_imu_state test_imu_state.cxx)
target_link_libraries(test_imu_state imu_state)

library(imu_model imu_model.cxx)
target_link_libraries(imu_model utctime imu_state check quantized_time)
add_gtest(test_imu_model test_imu_model.cxx)
target_link_libraries(test_imu_model imu_model logger header)
add_benchmark(bench_imu_model SRCS bench_imu_model.cxx DEPS imu_model)

library(prange_model prange_model.cxx)
target_link_libraries(prange_model atm_correction satellite check)
add_gtest(test_prange_model test_prange_model.cxx)
target_link_libraries(test_prange_model prange_model sdr_adapter gps)
add_benchmark(bench_prange_model SRCS bench_prange_model.cxx DEPS prange_model gps)

library(gnss_dynamics_model gnss_dynamics_model.cxx)
target_link_libraries(gnss_dynamics_model utctime check print)
add_gtest(test_gnss_dynamics_model test_gnss_dynamics_model.cxx)
target_link_libraries(test_gnss_dynamics_model gnss_dynamics_model)
add_benchmark(bench_gnss_dynamics_model SRCS bench_gnss_dynamics_model.cxx DEPS gnss_dynamics_model)

library(pose_error_model pose_error_model.cxx)
target_link_libraries(pose_error_model utctime check print)
add_gtest(test_pose_error_model test_pose_error_model.cxx)
target_link_libraries(test_pose_error_model pose_error_model)
add_benchmark(bench_pose_error_model SRCS bench_pose_error_model.cxx DEPS pose_error_model)

library(relative_carrier_phase_model relative_carrier_phase_model.cxx)
target_link_libraries(relative_carrier_phase_model utctime check print)
add_gtest(test_relative_carrier_phase_model test_relative_carrier_phase_model.cxx)
target_link_libraries(test_relative_carrier_phase_model
    relative_carrier_phase_model
    wgs84
    gps
    satellite)
add_benchmark(bench_relative_carrier_phase_model
    SRCS bench_relative_carrier_phase_model.cxx
    DEPS relative_carrier_phase_model wgs84 gps satellite)

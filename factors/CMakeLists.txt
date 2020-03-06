library(imu_state imu_state.cxx)

library(imu_integrator imu_integrator.cxx)
target_link_libraries(imu_integrator utctime imu_state)


add_gtest(test_imu_integrator test_imu_integrator.cxx)
target_link_libraries(test_imu_integrator imu_integrator)

add_benchmark(bench_imu_integrator SRCS bench_imu_integrator.cxx DEPS imu_integrator)

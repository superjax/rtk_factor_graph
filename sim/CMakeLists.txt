add_subdirectory(sensors)
add_subdirectory(dynamics)
add_subdirectory(controllers)

library(sim sim.cxx)
target_link_libraries(sim car gnss imu)
add_gtest(test_sim test_sim.cxx)
target_link_libraries(test_sim sim)

add_subdirectory(sensors)
add_subdirectory(dynamics)
add_subdirectory(controllers)

library(sim sim.cxx)
target_link_libraries(
    sim
    car
    gnss
    imu
)

executable(simulate simulate.cxx)
target_link_libraries(
    simulate
    sim
    log_reader
    log_writer
    progress_bar
    config
)

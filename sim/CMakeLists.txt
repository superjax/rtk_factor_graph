add_subdirectory(sensors)
add_subdirectory(dynamics)
add_subdirectory(controllers)

library(sim sim.cxx)
target_link_libraries(sim car gnss imu)


executable(simulate simulate.cxx)
target_link_libraries(simulate sim header logger progress_bar)

py_executable(plot_sim plot_sim.py)

library(imu imu.cxx)

library(gnss gnss.cxx)
target_link_libraries(
    gnss
    satellite
    atm_correction
    log_reader
)

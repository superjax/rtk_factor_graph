library(gps_sdr_sim gps_sdr_sim.cxx)
target_link_libraries(gps_sdr_sim print)

library(sdr_adapter sdr_adapter.cxx)
target_link_libraries(sdr_adapter gps_sdr_sim)

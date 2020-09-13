executable(gather_ephemeris gather_ephemeris.cxx)
target_link_libraries(gather_ephemeris async_comm print ubx gps glonass galileo logger header)

executable(verify_ephemeris verify_ephemeris.cxx)
target_link_libraries(verify_ephemeris print gps glonass galileo log_reader)

py_executable(plot_gathered_eph plot_gathered_eph.py)

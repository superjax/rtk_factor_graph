library(dynamics dynamics.cxx)
target_link_libraries(dynamics utctime print)

library(car car.cxx)
target_link_libraries(car dynamics print pid)
add_gtest(test_car test_car.cxx)
target_link_libraries(test_car car logger)

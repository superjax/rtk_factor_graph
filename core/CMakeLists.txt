include_directories(..)

add_subdirectory(factors)
add_subdirectory(states)
add_subdirectory(solver)

library(factor_graph factor_graph.cxx)
target_link_libraries(factor_graph utctime)
add_gtest(test_factor_graph test_factor_graph.cxx)
target_link_libraries(test_factor_graph factor_graph)

library(estimator estimator.cxx)
target_link_libraries(estimator factor_graph)
add_gtest(test_estimator test_estimator.cxx)
target_link_libraries(test_estimator estimator)

library(sat_manager sat_manager.cxx)
target_link_libraries(sat_manager satellite)
add_gtest(test_sat_manager test_sat_manager.cxx)
target_link_libraries(test_sat_manager sat_manager)

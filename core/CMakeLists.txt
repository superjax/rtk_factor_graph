include_directories(..)

add_subdirectory(factors)
add_subdirectory(states)
add_subdirectory(solver)


library(factor_graph factor_graph.cxx)

add_gtest(test_factor_graph test_factor_graph.cxx)
target_link_libraries(test_factor_graph factor_graph)

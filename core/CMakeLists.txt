include_directories(..)

add_subdirectory(factors)
add_subdirectory(states)
add_subdirectory(solver)


add_gtest(test_factor_graph test_factor_graph.cxx)
library(factor_graph factor_graph.cxx)
target_link_libraries(factor_graph)

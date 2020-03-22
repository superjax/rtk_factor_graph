add_gtest(test_solver test_solver.cxx)
library(solver solver.cxx)
target_link_libraries(solver)
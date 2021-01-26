add_gtest(test_solver test_solver.cxx)
library(solver solver.cxx)
target_link_libraries(solver)

library(lambda lambda.cxx)
target_link_libraries(lambda check print)
add_gtest(test_lambda test_lambda.cxx)
target_link_libraries(test_lambda lambda)
add_benchmark(
    bench_lambda
    SRCS bench_lambda.cxx
    DEPS lambda
)

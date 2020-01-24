add_gtest(test_so3 test_so3.cxx)
add_gtest(test_se3 test_se3.cxx)
add_gtest(test_quat test_quat.cxx)
add_gtest(test_dquat test_dquat.cxx)

add_benchmark(bench_quat SRCS bench_quat.cxx)
add_benchmark(bench_so3 SRCS bench_so3.cxx)
add_benchmark(bench_se3 SRCS bench_se3.cxx)
add_benchmark(bench_dquat SRCS bench_dquat.cxx)

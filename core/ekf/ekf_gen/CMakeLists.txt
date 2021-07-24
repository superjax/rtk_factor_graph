py_executable(ekf_gen ekf_gen.py)

ekf_cpp(example_ekf example_ekf.yaml)

add_gtest(example_ekf_test example_ekf_test.cxx)
target_link_libraries(example_ekf_test example_ekf)

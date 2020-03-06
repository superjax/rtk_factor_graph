add_subdirectory(gitlab_ci)
add_subdirectory(third_party_install)
add_subdirectory(benchmark)

include_directories(..)
include_directories(${EIGEN3_INCLUDE_DIRS})
add_executable(test test.cxx)
add_dependencies(test eigen_ext)

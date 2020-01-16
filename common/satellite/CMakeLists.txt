include_directories(../..)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_library(azel azel.cxx)
add_dependencies(azel eigen_ext)
target_link_libraries(azel wgs84)

add_gtest(test_azel SRCS test_azel.cxx)
target_link_libraries(test_azel azel)

add_library(satellite_state satellite_state.cxx)
add_dependencies(satellite_state eigen_ext)
target_link_libraries(satellite_state glonass utctime)

add_gtest(test_satellite_state SRCS test_satellite_state.cxx)
target_link_libraries(test_satellite_state satellite_state rtklib rtklib_adapter)

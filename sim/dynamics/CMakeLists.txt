library(dynamics dynamics.cxx)
target_link_libraries(dynamics utctime print)

library(car car.cxx)
target_link_libraries(car dynamics print pid)
add_gtest(test_car test_car.cxx)
target_link_libraries(test_car car logger)

add_custom_command(OUTPUT plot_test_car_script
    COMMAND echo "#!/bin/bash" > ${PROJECT_BINARY_DIR}/plot_test_car3
    COMMENT "Creating ${PROJECT_BINARY_DIR}/plot_test_car3"
    OUTPUT ${PROJECT_BINARY_DIR}/plot_test_car3
)

add_custom_target(plot_test_car2 ALL
    DEPENDS plot_test_car_script
    SOURCES plot_test_car.py
)

py_executable(plot_test_car plot_test_car.py)

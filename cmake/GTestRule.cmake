file(REMOVE "${PROJECT_BINARY_DIR}/run_tests")
file(WRITE  "${PROJECT_BINARY_DIR}/run_tests" "#!/bin/bash\n\n")
file(REMOVE "${PROJECT_BINARY_DIR}/test_list")
file(REMOVE "${PROJECT_BINARY_DIR}/test_list")

set(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")

function(add_gtest name)
    add_executable(${name} ${ARGV})
    target_link_libraries(${name} ${GTEST_LIBRARY} gtest_main)
    add_dependencies(${name} eigen_ext)
    target_include_directories(${name} PUBLIC
                               ${GTEST_INCLUDE_DIRS}
                               ${REPO_ROOT}
                               ${EIGEN3_INCLUDE_DIRS})
    add_custom_command(TARGET ${name} POST_BUILD
                       COMMAND ${CMAKE_COMMAND} -E echo $<TARGET_FILE:${name}> >> ${PROJECT_BINARY_DIR}/run_tests && chmod +x ${PROJECT_BINARY_DIR}/run_tests)
    add_custom_command(TARGET ${name} POST_BUILD
                       COMMAND ${CMAKE_COMMAND} -E echo $<TARGET_FILE:${name}> >> ${PROJECT_BINARY_DIR}/test_list)
endfunction()

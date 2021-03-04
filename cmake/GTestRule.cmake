file(REMOVE "${PROJECT_BINARY_DIR}/run_tests")
file(WRITE "${PROJECT_BINARY_DIR}/run_tests" "#!/bin/bash\n\n")
file(REMOVE "${PROJECT_BINARY_DIR}/test_list")
file(REMOVE "${PROJECT_BINARY_DIR}/test_list")

set(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")

function(add_gtest name)
    add_executable(${name} ${ARGV})
    target_link_libraries(
        ${name}
        ${GTEST_LIBRARY}
        ${GMOCK_LIBRARY}
        gtest_main
        stdc++fs
    )
    add_dependencies(${name} eigen_ext fmt)
    target_include_directories(
        ${name}
        PUBLIC ${REPO_ROOT}
               ${EIGEN3_INCLUDE_DIRS}
               ${FMT_INCLUDE_DIRS}
               ${GTEST_INCLUDE_DIRS}
               ${GMOCK_INCLUDE_DIRS}
    )
    add_custom_command(
        TARGET ${name}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo $<TARGET_FILE:${name}> >> ${PROJECT_BINARY_DIR}/run_tests
                && chmod +x ${PROJECT_BINARY_DIR}/run_tests
    )
    add_custom_command(
        TARGET ${name}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo $<TARGET_FILE:${name}> >> ${PROJECT_BINARY_DIR}/test_list
    )
endfunction()

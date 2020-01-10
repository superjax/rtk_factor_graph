include(CMakeParseArguments)

file(REMOVE "${PROJECT_BINARY_DIR}/run_tests")
file(WRITE  "${PROJECT_BINARY_DIR}/run_tests" "#!/bin/bash\n\n")
file(REMOVE "${PROJECT_BINARY_DIR}/test_list")
file(REMOVE "${PROJECT_BINARY_DIR}/test_list")

function(add_gtest name)
    cmake_parse_arguments(
        PARSED_ARGS # prefix of output variables
        "" # list of names of the boolean arguments (only defined ones will be true)
        "" # list of names of mono-valued arguments
        "SRCS;DEPS" # list of names of multi-valued arguments (output variables are lists)
        ${ARGN} # arguments of the function to parse, here we take the all original ones
    )
    add_executable(${name} ${PARSED_ARGS_SRCS})
    target_link_libraries(${name} ${GTEST_LIBRARY} gtest_main)
    target_link_libraries(${name} ${PARSED_ARGS_DEPS})
    target_include_directories(${name} PUBLIC ${GTEST_INCLUDE_DIRS})
    add_custom_command(TARGET ${name} POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E echo $<TARGET_FILE:${name}> >> ${PROJECT_BINARY_DIR}/run_tests && chmod +x ${PROJECT_BINARY_DIR}/run_tests)
    add_custom_command(TARGET ${name} POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E echo $<TARGET_FILE:${name}> >> ${PROJECT_BINARY_DIR}/test_list)
endfunction()

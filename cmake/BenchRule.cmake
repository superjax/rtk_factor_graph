include(CMakeParseArguments)

file(REMOVE "${PROJECT_BINARY_DIR}/run_benchmarks")
file(REMOVE "${PROJECT_BINARY_DIR}/benchmark_list")
if (BUILD_BENCHMARKS)
message("Building Benchmarks")
file(WRITE  "${PROJECT_BINARY_DIR}/run_benchmarks" "#!/bin/bash\n\nmkdir -p bench_results\n\n")
file(MAKE_DIRECTORY bench_results)
endif()
set(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")

function(add_benchmark name)
if(BUILD_BENCHMARKS)
cmake_parse_arguments(
        PARSED_ARGS #prefix of output variables
        "" #list of names of the boolean arguments(only defined ones will be true)
        "" #list of names of mono - valued arguments
        "SRCS;DEPS" #list of names of multi - valued arguments(output variables are lists)
        ${ARGN} #arguments of the function to parse, here we take the all original ones
    )

    add_executable(${name} ${PARSED_ARGS_SRCS})
    target_link_libraries(${name}
                          ${GBENCHMARK_MAIN_LIBRARY}
                          ${GBENCHMARK_LIBRARY}
                          ${PARSED_ARGS_DEPS})
    add_dependencies(${name} eigen_ext google_benchmark_ext)
    target_include_directories(${name} PUBLIC
                               ${GBENCHMARK_INCLUDE_DIRS}
                               ${REPO_ROOT}
                               ${EIGEN3_INCLUDE_DIRS})
    add_custom_command(TARGET ${name} POST_BUILD
                       COMMAND ${CMAKE_COMMAND} -E echo "$<TARGET_FILE:${name}> --benchmark_out=bench_results/$<TARGET_PROPERTY:${name},NAME>.json"  >> ${PROJECT_BINARY_DIR}/run_benchmarks && chmod +x ${PROJECT_BINARY_DIR}/run_benchmarks)
    add_custom_command(TARGET ${name} POST_BUILD
                       COMMAND ${CMAKE_COMMAND} -E echo $<TARGET_FILE:${name}> >> ${PROJECT_BINARY_DIR}/benchmark_list)
endif()
endfunction()

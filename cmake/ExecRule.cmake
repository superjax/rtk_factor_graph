set(REPO_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)

function(executable name)
    add_executable(${name} ${ARGV})
    add_dependencies(${name} eigen_ext)
    add_dependencies(${name} fmt_ext)
    add_dependencies(${name} async_comm)
    target_include_directories(
        ${name}
        PUBLIC ${EIGEN3_INCLUDE_DIRS}
               ${FMT_INCLUDE_DIRS}
               ${ASYNC_COMM_INCLUDE_DIRS}
               ${YAML_INCLUDE_DIRS}
               ${GTEST_INCLUDE_DIRS}
               ${GMOCK_INCLUDE_DIRS}
               ${REPO_ROOT}
    )
    set_target_properties(${name} PROPERTIES LINKER_LANGUAGE CXX)
endfunction()

set(REPO_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)

function(library name)
    add_library(${name} ${ARGV})
    add_dependencies(${name} eigen_ext)
    add_dependencies(${name} fmt_ext)
    target_include_directories(${name} PUBLIC
        ${EIGEN3_INCLUDE_DIRS}
        ${FMT_INCLUDE_DIRS}
        ${REPO_ROOT})
    target_link_libraries(${name})
    set_target_properties(${name} PROPERTIES LINKER_LANGUAGE CXX)
endfunction()

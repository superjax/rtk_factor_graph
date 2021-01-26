function(capnp_cpp name src)
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${src}.h ${CMAKE_CURRENT_BINARY_DIR}/${src}.c++
        COMMAND
            ${CMAKE_COMMAND} -E env "PATH=${CAPNP_BINARY_PATH}:$ENV{PATH}" capnp compile
            -oc++:${CMAKE_CURRENT_BINARY_DIR} --src-prefix=${CMAKE_CURRENT_LIST_DIR}
            ${CMAKE_CURRENT_LIST_DIR}/${src} -I ${CAPNP_INCLUDE_DIRS} -I ${REPO_ROOT}
        DEPENDS ext_capnp ${src}
    )
    add_library(${name} ${CMAKE_CURRENT_BINARY_DIR}/${src}.c++)
    target_link_libraries(${name} capnp kj)
    target_include_directories(
        ${name} PUBLIC ${CAPNP_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} ${PROJECT_BINARY_DIR}
    )
endfunction()

function(ekf_cpp name src)
    string(
        REGEX
        REPLACE "\\.[^.]*$"
                ""
                SRC_WITHOUT_EXT
                ${src}
    )
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${SRC_WITHOUT_EXT}.h
               ${CMAKE_CURRENT_BINARY_DIR}/${SRC_WITHOUT_EXT}.cxx
               ${CMAKE_CURRENT_BINARY_DIR}/${SRC_WITHOUT_EXT}_sparse_test.cxx
               ${CMAKE_CURRENT_BINARY_DIR}/${SRC_WITHOUT_EXT}_state.cxx
               ${CMAKE_CURRENT_BINARY_DIR}/${SRC_WITHOUT_EXT}_state.h
               ${CMAKE_CURRENT_BINARY_DIR}/${SRC_WITHOUT_EXT}_types.cxx
               ${CMAKE_CURRENT_BINARY_DIR}/${SRC_WITHOUT_EXT}_types.h
        COMMAND
            ${CMAKE_COMMAND} -E env PYTHONPATH="${REPO_ROOT}:$ENV{PYTHONPATH}" python3
            ${REPO_ROOT}/core/ekf/ekf_gen/ekf_gen.py --config_file ${CMAKE_CURRENT_LIST_DIR}/${src}
            --destination ${CMAKE_CURRENT_BINARY_DIR}
        DEPENDS ${src}
                ${REPO_ROOT}/core/ekf/ekf_gen/ekf_gen.py
                ${REPO_ROOT}/core/ekf/ekf_gen/ekf_gen_matrix_manip.py
                ${REPO_ROOT}/core/ekf/ekf_gen/ekf_gen_state.py
                ${REPO_ROOT}/core/ekf/ekf_gen/ekf_gen_templates.py
                ${REPO_ROOT}/core/ekf/ekf_gen/ekf_gen_utils.py
                ${REPO_ROOT}/core/ekf/ekf_gen/symb_algebra.py
                ${REPO_ROOT}/core/ekf/ekf_gen/symb_block_matrix.py
                ${REPO_ROOT}/core/ekf/ekf_gen/symb_matrix.py
                ${REPO_ROOT}/core/ekf/ekf_gen/ekf_gen_test.py
    )

    library(${name}_state ${CMAKE_CURRENT_BINARY_DIR}/${SRC_WITHOUT_EXT}_state.cxx)
    target_include_directories(${name}_state PUBLIC ${CMAKE_BINARY_DIR})
    target_link_libraries(
        ${name}_state
        utctime
        quantized_time
        check
    )

    library(${name}_types ${CMAKE_CURRENT_BINARY_DIR}/${SRC_WITHOUT_EXT}_types.cxx)
    target_include_directories(${name}_types PUBLIC ${CMAKE_BINARY_DIR})
    target_link_libraries(${name}_types ${name}_state)

    library(${name} ${CMAKE_CURRENT_BINARY_DIR}/${SRC_WITHOUT_EXT}.cxx)
    target_include_directories(${name} PUBLIC ${CMAKE_BINARY_DIR})
    target_link_libraries(${name} ${name}_types)

    add_gtest(${name}_sparse_test ${name}_sparse_test.cxx)
    target_link_libraries(${name}_sparse_test ${name})
endfunction()

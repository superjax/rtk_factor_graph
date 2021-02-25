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
               ${CMAKE_CURRENT_BINARY_DIR}/state.cxx ${CMAKE_CURRENT_BINARY_DIR}/state.h
        COMMAND
            ${CMAKE_COMMAND} -E env PYTHONPATH="${REPO_ROOT}:$ENV{PYTHONPATH}" python3
            ${REPO_ROOT}/core/ekf/ekf_gen/ekf_gen.py --config_file ${CMAKE_CURRENT_LIST_DIR}/${src}
            --destination ${CMAKE_CURRENT_BINARY_DIR}
        DEPENDS ${src}
                ${REPO_ROOT}/core/ekf/ekf_gen/ekf_gen.py
                ${REPO_ROOT}/core/ekf/ekf_gen/ekf_gen_utils.py
                ${REPO_ROOT}/core/ekf/ekf_gen/ekf_gen_matrix_manip.py
                ${REPO_ROOT}/core/ekf/ekf_gen/ekf_gen_templates.py
                ${REPO_ROOT}/core/ekf/ekf_gen/symb_algebra.py
                ${REPO_ROOT}/core/ekf/ekf_gen/symb_block_matrix.py
                ${REPO_ROOT}/core/ekf/ekf_gen/symb_matrix.py
    )
    library(${name} ${CMAKE_CURRENT_BINARY_DIR}/${SRC_WITHOUT_EXT}.h
            ${CMAKE_CURRENT_BINARY_DIR}/state.cxx
    )
    target_include_directories(${name} PUBLIC ${CMAKE_BINARY_DIR})
    target_link_libraries(${name} utctime)
endfunction()

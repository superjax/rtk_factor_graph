function(py_executable name src)
    set(DESTINATION_SCRIPT ${CMAKE_CURRENT_BINARY_DIR}/${name})
    set(PYTHON_FILE ${CMAKE_CURRENT_LIST_DIR}/${src})

    # This creates a script that runs the python file after adding the root of this project to the
    # pythonpath
    add_custom_target(${name} ALL
        COMMAND ${CMAKE_COMMAND} -E echo "#!/bin/bash" > ${DESTINATION_SCRIPT}
        COMMAND ${CMAKE_COMMAND} -E echo "export PYTHONPATH=\$PYTHONPATH:${PROJECT_SOURCE_DIR}" >> ${DESTINATION_SCRIPT}
        COMMAND ${CMAKE_COMMAND} -E echo "python3 ${PYTHON_FILE}" >> ${DESTINATION_SCRIPT}
        COMMAND chmod +x ${DESTINATION_SCRIPT}
        BYPRODUCTS ${DESTINATION_SCRIPT}
        VERBATIM
    )
endfunction()

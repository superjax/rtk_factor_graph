include(CMakeParseArguments)

function(add_gtest name)
    cmake_parse_arguments(
        PARSED_ARGS # prefix of output variables
        "" # list of names of the boolean arguments (only defined ones will be true)
        "" # list of names of mono-valued arguments
        "SRCS;DEPS" # list of names of multi-valued arguments (output variables are lists)
        ${ARGN} # arguments of the function to parse, here we take the all original ones
    )
    # note: if it remains unparsed arguments, here, they can be found in variable PARSED_ARGS_UNPARSED_ARGUMENTS
    #message("NAME: ${name}")
    #message("Provided sources are:")
    #foreach(src ${PARSED_ARGS_SRCS})
        #message("- ${src}")
    #endforeach(src)
    add_executable(${name} ${PARSED_ARGS_SRCS})
    target_link_libraries(${name} ${GTEST_LIBRARY} gtest_main)
    target_include_directories(${name} PUBLIC ${GTEST_INCLUDE_DIRS})
endfunction()

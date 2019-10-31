include(FetchContent)

function(FetchContentHelper name vc url tag)
    set(options ADD_SUBDIR)
    set(mvargs CONFIG_SUBDIR)
    cmake_parse_arguments(PARSE_ARGV 4 FCH "${options}" "" "${mvargs}")

    FetchContent_Declare(
        ${name}
        ${vc}_REPOSITORY ${url}
        ${vc}_TAG        ${tag}
        GIT_PROGRESS     ON
    )
    FetchContent_GetProperties(${name})
    if(NOT ${name}_POPULATED)
        message("Setting up ${name} from ${url}")
        FetchContent_Populate(${name})
        if(FCH_ADD_SUBDIR)
            foreach(config ${FCH_CONFIG_SUBDIR})
                string(REPLACE "=" ";" configkeyval ${config})
                list(LENGTH configkeyval len)
                if (len GREATER_EQUAL 2)
                    list(GET configkeyval 0 configkey)
                    list(SUBLIST configkeyval 1 -1 configvals)
                    string(REPLACE ";" "=" configval "${configvals}")
                else()
                    message(FATAL_ERROR "Invalid config: ${configkeyval}")
                endif()
                message("Set ${configkey} = ${configval}")
                set(${configkey} ${configval} CACHE INTERNAL "" FORCE)
            endforeach()
            set(${name}_SOURCE_DIR ${${name}_SOURCE_DIR} PARENT_SCOPE)
            set(${name}_BINARY_DIR ${${name}_BINARY_DIR} PARENT_SCOPE)
            add_subdirectory(${${name}_SOURCE_DIR} ${${name}_BINARY_DIR} EXCLUDE_FROM_ALL)
        endif()
    endif()
endfunction(FetchContentHelper)
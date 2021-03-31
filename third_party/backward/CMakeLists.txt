find_library(LIBDW_LIBRARY dw)
library(backward backward.cxx)

target_include_directories(backward PRIVATE ${LIBDW_INCLUDE_DIRS})
if(${FANCY_BACKTRACE})
    target_link_libraries(backward dw)
endif()

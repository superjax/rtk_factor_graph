set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp --std=c++17")

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache) # Less useful to do it for linking, see edit2
endif(CCACHE_FOUND)

# C++ strict compilation
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wstrict-aliasing -Wformat=2 -Winit-self \
    -Wlogical-op -Wmissing-include-dirs -Wnoexcept -Wunreachable-code -Wcast-align -Wcast-qual \
    -Wdisabled-optimization -Woverloaded-virtual -Wredundant-decls -Wshadow -Wsign-promo \
    -Wstrict-null-sentinel -Wstrict-overflow=1 -Wno-address-of-packed-member")
# Extra features/enablements
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Set all warnings as errors
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")

# Test Coverage Compile Flags
if (TEST_COVERAGE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg -coverage -fprofile-arcs")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fprofile-arcs")
endif()

if (DISABLE_CHECK)
    message("Disabling Checks")
    add_definitions(-DDISABLE_CHECK)
endif()

if (BUILD_BENCHMARKS)
    message("Disabling Checks")
    add_definitions(-DDISABLE_CHECK)
    add_definitions(-DLOG_LEVEL=10)
endif()

if (LOG_LEVEL)
    add_definitions(-DLOG_LEVEL=${LOG_LEVEL})
endif()

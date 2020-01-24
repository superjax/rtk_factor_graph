set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")


# C++ strict compilation
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wstrict-aliasing -Wformat=2 -Winit-self \
    -Wlogical-op -Wmissing-include-dirs -Wnoexcept -Wunreachable-code -Wcast-align -Wcast-qual \
    -Wdisabled-optimization -Woverloaded-virtual -Wredundant-decls -Wshadow -Wsign-promo \
    -Wstrict-null-sentinel -Wstrict-overflow=1")
# Extra features/enablements
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Set all warnings as errors
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")

# Test Coverage Compile Flags
if (TEST_COVERAGE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg -coverage -fprofile-arcs")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fprofile-arcs")
endif()

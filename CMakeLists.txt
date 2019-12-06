cmake_minimum_required(VERSION 3.14)
project(globalai)


# Project-Wide Settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")



# Third-Party Libraries
include(ExternalProject)
include(cmake/External_Eigen3.cmake)
include(cmake/External_GTest.cmake)
# include(cmake/External_GFlags.cmake)
# include(cmake/External_GLog.cmake)
# include(cmake/External_Ceres.cmake)
include(cmake/GTestRule.cmake)

# Strict C++ compilation
include(cmake/StrictCompilation.cmake)

# Project Code
add_subdirectory(scratch)
add_subdirectory(common)
add_subdirectory(utils)
add_subdirectory(client)

add_subdirectory(third_party)

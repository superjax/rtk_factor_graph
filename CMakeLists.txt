cmake_minimum_required(VERSION 3.14)
project(globalai)


# Project-Wide Settings
include(cmake/Options.cmake)
include(cmake/CleanRule.cmake)
include(cmake/CompileFlags.cmake)

include(cmake/CleanRule.cmake)

# Third-Party Libraries
include(ExternalProject)
include(cmake/External_Eigen3.cmake)
include(cmake/External_GTest.cmake)
# include(cmake/External_GFlags.cmake)
# include(cmake/External_GLog.cmake)
# include(cmake/External_Ceres.cmake)
include(cmake/GTestRule.cmake)


# Project Code
add_subdirectory(scratch)
add_subdirectory(common)
add_subdirectory(utils)
add_subdirectory(client)

add_subdirectory(third_party)

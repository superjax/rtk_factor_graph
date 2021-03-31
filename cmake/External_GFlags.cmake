find_package(Threads REQUIRED)

ExternalProject_Add(
    google_gflags
    GIT_REPOSITORY https://github.com/gflags/gflags.git
    GIT_TAG v2.2.0
    UPDATE_COMMAND ""
    INSTALL_COMMAND ""
    LOG_DOWNLOAD ON
    GIT_SHALLOW ON
    LOG_CONFIGURE ON
    LOG_BUILD ON
    CMAKE_ARGS -DCMAKE_BUILD_TYPE=RelWithDebInfo
)

ExternalProject_Get_Property(google_gflags binary_dir)
ExternalProject_Get_Property(google_gflags source_dir)
set(GFLAGS_INCLUDE_DIRS ${binary_dir} ${source_dir}/src)

ExternalProject_Get_Property(google_gflags binary_dir)
set(GFLAGS_LIBRARY_PATH ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}glog.a)
set(GFLAGS_LIBRARY gflags)
add_library(${GFLAGS_LIBRARY} UNKNOWN IMPORTED)
set_target_properties(${GFLAGS_LIBRARY} PROPERTIES IMPORTED_LOCATION ${GFLAGS_LIBRARY_PATH})
add_dependencies(${GTEST_LIBRARY} google_gflags)

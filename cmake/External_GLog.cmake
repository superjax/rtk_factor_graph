find_package(Threads REQUIRED)

ExternalProject_Add(
  google_glog
  GIT_REPOSITORY https://github.com/google/glog.git
  GIT_TAG        v0.4.0
  UPDATE_COMMAND ""
  INSTALL_COMMAND ""
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  LOG_BUILD ON
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=RelWithDebInfo)

ExternalProject_Get_Property(google_glog binary_dir)
ExternalProject_Get_Property(google_glog source_dir)
set(GLOG_INCLUDE_DIRS ${binary_dir} ${source_dir}/src)

ExternalProject_Get_Property(google_glog binary_dir)
set(GLOG_LIBRARY_PATH ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}glog.a)
set(GLOG_LIBRARY glog)
add_library(${GLOG_LIBRARY} UNKNOWN IMPORTED)
set_target_properties(${GLOG_LIBRARY} PROPERTIES
  IMPORTED_LOCATION ${GLOG_LIBRARY_PATH}
  IMPORTED_LINK_INTERFACE_LIBRARIES ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(${GLOG_LIBRARY} google_glog)

ExternalProject_Add(
  ext_ceres
  GIT_REPOSITORY https://github.com/ceres-solver/ceres-solver.git
  GIT_TAG        2.0.0
  UPDATE_COMMAND ""
  INSTALL_COMMAND ""
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  LOG_BUILD ON
  GIT_SHALLOW ON
  CMAKE_ARGS -Dgflags_DIR=${GFLAGS_INCLUDE_DIRS}
             -DGLOG_INCLUDE_DIRS=${GLOGS_INCLUDE_DIRS}
             -DBUILD_EXAMPLES=OFF
             -DBUILD_TESTING=OFF
             -DBUILD_BENCHMARKS=OFF
             -DCMAKE_BUILD_TYPE=RelWithDebInfo
  PATCH_COMMAND git reset --hard && git apply ${CMAKE_CURRENT_LIST_DIR}/patches/ceres.patch
)

ExternalProject_Get_Property(ext_ceres binary_dir)
ExternalProject_Get_Property(ext_ceres source_dir)
set(CERES_INCLUDE_DIRS ${source_dir}/include ${binary_dir}/config ${EIGEN3_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS})

ExternalProject_Get_Property(ext_ceres binary_dir)
set(CERES_LIBRARY_PATH ${binary_dir}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}ceres.a)
set(CERES_LIBRARY ceres)
add_library(${CERES_LIBRARY} UNKNOWN IMPORTED)
set_target_properties(${CERES_LIBRARY} PROPERTIES
  IMPORTED_LOCATION ${CERES_LIBRARY_PATH})
add_dependencies(ext_ceres eigen_ext)
add_dependencies(ext_ceres ${GFLAGS_LIBRARY})
add_dependencies(ext_ceres ${GLOG_LIBRARY})
add_dependencies(${CERES_LIBRARY} ext_ceres)
set(CERES_LIBRARIES glog pthread)

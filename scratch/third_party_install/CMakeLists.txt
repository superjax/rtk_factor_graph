add_gtest(eigen_install SRCS eigen_install.cxx)
target_include_directories(eigen_install PUBLIC ${EIGEN3_INCLUDE_DIRS})
add_dependencies(eigen_install eigen_ext)

# add_executable(glog_install glog_install.cxx)
# target_include_directories(glog_install PUBLIC ${GLOG_INCLUDE_DIRS})
# target_link_libraries(glog_install glog)
# add_dependencies(glog_install glog)

# add_gtest(ceres_install SRCS ceres_install.cxx)
# target_include_directories(ceres_install PUBLIC ${CERES_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS})
# target_link_libraries(ceres_install ceres glog pthread)
# add_dependencies(ceres_install ceres)

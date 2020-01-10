add_gtest(test_eigen_install SRCS test_eigen_install.cxx)
target_include_directories(test_eigen_install PUBLIC ${EIGEN3_INCLUDE_DIRS})
add_dependencies(test_eigen_install eigen_ext)

# add_executable(test_glog_install test_glog_install.cxx)
# target_include_directories(test_glog_install PUBLIC ${GLOG_INCLUDE_DIRS})
# target_link_libraries(test_glog_install glog)
# add_dependencies(test_glog_install glog)

# add_gtest(test_ceres_install SRCS test_ceres_install.cxx)
# target_include_directories(test_ceres_install PUBLIC ${CERES_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS})
# target_link_libraries(test_ceres_install ceres glog pthread)
# add_dependencies(test_ceres_install ceres)

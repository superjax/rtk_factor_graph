ExternalProject_Add(
  eigen_ext
  GIT_REPOSITORY https://github.com/eigenteam/eigen-git-mirror.git
  GIT_TAG        3.3.7
  UPDATE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON)

ExternalProject_Get_Property(eigen_ext source_dir)
ExternalProject_Get_Property(eigen_ext binary_dir)
message(EIGEN SOURCE DIR = ${source_dir})
message(EIGEN BINARY DIR = ${binary_dir})
set(EIGEN3_INCLUDE_DIRS ${source_dir})
set(EIGEN_INCLUDE_DIR ${source_dir})
set(EIGEN_INCLUDE_DIRS ${source_dir})
set(EIGEN_INCLUDE_DIR_HINTS ${source_dir})
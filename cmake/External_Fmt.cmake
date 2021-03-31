ExternalProject_Add(
  fmt_ext
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG        6.1.2
  UPDATE_COMMAND ""
  INSTALL_COMMAND ""
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  GIT_SHALLOW ON
  LOG_BUILD ON
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=RelWithDebInfo -DFMT_DOC=OFF -DFMT_INSTALL=OFF -DFMT_TEST=OFF
)

ExternalProject_Get_Property(fmt_ext binary_dir)
ExternalProject_Get_Property(fmt_ext source_dir)

set(FMT_INCLUDE_DIRS ${source_dir}/include)
set(FMT_LIBRARY_PATH ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}fmt.a)
set(FMT_LIBRARY fmt)
add_library(${FMT_LIBRARY} UNKNOWN IMPORTED)
set_target_properties(${FMT_LIBRARY} PROPERTIES
  IMPORTED_LOCATION ${FMT_LIBRARY_PATH})
add_dependencies(${FMT_LIBRARY} fmt_ext)

find_package(Threads REQUIRED)

if (BUILD_BENCHMARKS)
ExternalProject_Add(
  google_benchmark_ext
  GIT_REPOSITORY https://github.com/google/benchmark.git
  GIT_TAG        v1.5.0
  UPDATE_COMMAND ""
  INSTALL_COMMAND ""
  LOG_DOWNLOAD ON
  LOG_CONFIGURE ON
  LOG_BUILD ON
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBENCHMARK_ENABLE_GTEST_TESTS=OFF)

ExternalProject_Get_Property(google_benchmark_ext binary_dir)
ExternalProject_Get_Property(google_benchmark_ext source_dir)
set(GBENCHMARK_INCLUDE_DIRS ${binary_dir} ${source_dir}/include)
set(GBENCHMARK_LIBRARY_PATH ${binary_dir}/src/${CMAKE_FIND_LIBRARY_PREFIXES}benchmark.a)
set(GBENCHMARK_LIBRARY ${GBENCHMARK_LIBRARY_PATH})
add_library(benchmark UNKNOWN IMPORTED)
set_target_properties(benchmark PROPERTIES
  IMPORTED_LOCATION ${GBENCHMARK_LIBRARY_PATH}
  IMPORTED_LINK_INTERFACE_LIBRARIES ${CMAKE_THREAD_LIBS_INIT})

set(GBENCHMARK_MAIN_LIBRARY_PATH ${binary_dir}/src/${CMAKE_FIND_LIBRARY_PREFIXES}benchmark_main.a)
message ("GBENCHMARK_MAIN_LIBRARY_PATH = ${GBENCHMARK_MAIN_LIBRARY_PATH}")
set(GBENCHMARK_MAIN_LIBRARY benchmark_main)
add_library(${GBENCHMARK_MAIN_LIBRARY} UNKNOWN IMPORTED)
set_target_properties(${GBENCHMARK_MAIN_LIBRARY} PROPERTIES
  IMPORTED_LOCATION ${GBENCHMARK_MAIN_LIBRARY_PATH}
  IMPORTED_LINK_INTERFACE_LIBRARIES ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(${GBENCHMARK_MAIN_LIBRARY} benchmark)

endif()

find_package(Threads REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
ExternalProject_Add (
    async_comm_ext
    GIT_REPOSITORY https://github.com/dpkoch/async_comm.git
    GIT_TAG 0.2.0
    UPDATE_COMMAND ""
    INSTALL_COMMAND ""
    LOG_DOWNLOAD ON
    LOG_CONFIGURE ON
    LOG_BUILD ON
    GIT_SHALLOW ON
    PATCH_COMMAND git reset --hard && git apply ${CMAKE_CURRENT_LIST_DIR}/patches/async_comm.patch
    CMAKE_ARGS -DCMAKE_BUILD_TYPE=RelWithDebInfo
)

ExternalProject_Get_Property(async_comm_ext binary_dir)
ExternalProject_Get_Property(async_comm_ext source_dir)
set(ASYNC_COMM_INCLUDE_DIRS ${binary_dir} ${source_dir}/include)
set(ASYNC_COMM_LIBRARY_PATH ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}async_comm.so)
set(ASYNC_COMM_LIBRARY ${ASYNC_COMM_LIBRARY_PATH})
add_library(async_comm UNKNOWN IMPORTED)
set_target_properties(async_comm PROPERTIES
  IMPORTED_LOCATION ${ASYNC_COMM_LIBRARY_PATH}
  IMPORTED_LINK_INTERFACE_LIBRARIES ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(async_comm async_comm_ext)
target_link_libraries(async_comm INTERFACE ${Boost_LIBRARIES}  ${CMAKE_THREAD_LIBS_INIT})

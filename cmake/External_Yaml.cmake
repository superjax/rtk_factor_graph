ExternalProject_Add(
    yaml_ext
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG yaml-cpp-0.6.3
    UPDATE_COMMAND ""
    INSTALL_COMMAND ""
    PATCH_COMMAND git reset --hard && git apply ${CMAKE_CURRENT_LIST_DIR}/patches/yaml.patch
    LOG_DOWNLOAD ON
    LOG_CONFIGURE ON
    LOG_BUILD ON
    CMAKE_ARGS -DCMAKE_BUILD_TYPE=RelWithDebInfo -DYAML_CPP_BUILD_TESTS=OFF -DYAML_CPP_INSTALL=OFF
)

ExternalProject_Get_Property(yaml_ext binary_dir)
ExternalProject_Get_Property(yaml_ext source_dir)

set(YAML_INCLUDE_DIRS ${source_dir}/include)
set(YAML_LIBRARY_PATH ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}yaml-cpp.a)
set(YAML_LIBRARY ${YAML_LIBRARY_PATH})
add_library(yaml UNKNOWN IMPORTED)
set_target_properties(yaml PROPERTIES IMPORTED_LOCATION ${YAML_LIBRARY_PATH})

add_dependencies(yaml yaml_ext)

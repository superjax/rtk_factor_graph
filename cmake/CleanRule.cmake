# This script creates a file called clean in the build directory that removes everything except the
# third-party downloaded libraries.  This significantly speeds up clean builds

# First, create the script (This will need to be manually updated if we add source code in other
# directories)
file(WRITE "${PROJECT_BINARY_DIR}/tmp/clean"
     "#!/bin/bash\n\n rm -rf cmake_install.cmake\
                             coverage\
                             bench_results\
                             CMakeCache.txt\
                             CMakeFiles\
                             Makefile\
                             run_benchmarks\
                             run_tests\
                             client\
                             factors\
                             common\
                             scratch\
                             third_party\
                             utils\
                             tmp")

# Now, move the file into the build directory, and apply the right permisions.  You can't set the
# permissions on a new file through CMAKE, so this hack of starting in the /tmp folder, then moving
# it into the build will work for now
file(COPY "${PROJECT_BINARY_DIR}/tmp/clean"
    DESTINATION "${PROJECT_BINARY_DIR}"
    FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

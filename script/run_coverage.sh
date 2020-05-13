#!/bin/bash

function echo_bred    { echo -e "\033[1;31m$@\033[0m"; }
function echo_bgreen  { echo -e "\033[1;32m$@\033[0m"; }
function echo_bblue   { echo -e "\033[1;34m$@\033[0m"; }
function echo_red     { echo -e "\033[0;31m$@\033[0m"; }
function echo_green   { echo -e "\033[0;32m$@\033[0m"; }
function echo_blue    { echo -e "\033[0;34m$@\033[0m"; }

BASENAME=`basename "$PWD"`

if [ $BASENAME == "script" ]; then
    echo "cd ../build"
    cd ../build
elif [ $BASENAME != "build" ]; then
    echo "cd build"
    cd build
fi

BUILD_DIR=$PWD
ROOT=$PWD/..

# Adding BRANCH_COVERAGE makes things super slow
BRANCH_COVERAGE=$BRANCH_COVERAGE

rm -r coverage
mkdir coverage
rm stripped.info
rm all_tests.info

# Rebuild all tests with coverage
./clean
rm CMakeCache.txt
cmake .. -DCMAKE_BUILD_TYPE=Debug -DTEST_COVERAGE=ON
make -j4 -l4

while read p; do
    echo_bblue Gathering Coverage for $p
    # Run each test
    lcov --zerocounters --directory .
    $p
    if [[ "$(python3 -V)" =~ "Python 3" ]]; then
        lcov --capture --directory . --output-file ${p}.info.tmp --rc lcov_branch_coverage=$BRANCH_COVERAGE
        python3 ../script/fix_lcov_output.py --input ${p}.info.tmp --output ${p}.info
    else
        lcov --capture --directory . --output-file ${p}.info --rc lcov_branch_coverage=$BRANCH_COVERAGE
    fi
    if [[ -e all_tests.info ]]; then
        echo_bgreen "appending to tracefile"
        lcov --add-tracefile all_tests.info \
            --add-tracefile ${p}.info \
            --rc lcov_branch_coverage=$BRANCH_COVERAGE \
            --output-file all_tests.info
    else
        echo_bred "Starting new tracefile"
        lcov --add-tracefile ${p}.info \
            --rc lcov_branch_coverage=$BRANCH_COVERAGE \
            --output-file all_tests.info
    fi
done < test_list

function remove_files() {
    lcov --rc lcov_branch_coverage=$BRANCH_COVERAGE --remove all_tests.info "$1" --output-file all_tests.info
}

# Remove files that we don't care about
remove_files "/usr/**/*"
remove_files "*/Eigen/**/*"
remove_files "*/googletest/**/*"
remove_files "*/google_benchmark_ext-prefix/**/*"
remove_files "*/fmt_ext-prefix/**/*"
remove_files "*/**/bench_*"
remove_files "*/**/test_*"
remove_files "*/**/third_party/**/*"

lcov --list all_tests.info

genhtml --output-directory ./coverage --demangle-cpp \
    --num-spaces 2 --sort --title "GlobalAI Test Coverage" \
    --function-coverage --branch-coverage --legend \
    all_tests.info

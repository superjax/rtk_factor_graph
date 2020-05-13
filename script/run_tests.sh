#!/bin/bash

function echo_bred    { echo -e "\033[1;31m$@\033[0m"; }
function echo_bgreen  { echo -e "\033[1;32m$@\033[0m"; }
function echo_bblue   { echo -e "\033[1;34m$@\033[0m"; }
function echo_red     { echo -e "\033[0;31m$@\033[0m"; }
function echo_green   { echo -e "\033[0;32m$@\033[0m"; }
function echo_blue    { echo -e "\033[0;34m$@\033[0m"; }

EXIT_CODE=0
FAILED_TESTS=()
PASSED_TESTS=()

BASENAME=`basename "$PWD"`
echo $BASENAME
if [ $BASENAME == "script" ]; then
    echo "cd .."
    cd ..
elif [ $BASENAME == "build" ]; then
    echo "cd .."
    cd ..
fi

while read p; do
  echo_bblue Running Tests `basename "$p"`
  $p
  RESULT=$?
  if [ $RESULT -ne 0 ]; then
      FAILED_TESTS+=($p)
      EXIT_CODE=1
  else
      PASSED_TESTS+=($p)
  fi
done < build/test_list

printf "\n\nSUMMARY:\n-------------------------------------------------------------\n"
for t in "${PASSED_TESTS[@]}"; do
  echo_green "[PASSED]: "$t;
done
for t in "${FAILED_TESTS[@]}"; do
  echo_red "[FAILED]: "$t;
done


if [ $EXIT_CODE -eq 0 ]; then
  echo_bgreen "All tests passed!"
else
  echo_bred "There were failed tests"
fi

exit $EXIT_CODE

#!/bin/bash

function echo_red    { echo -e "\033[1;31m$@\033[0m"; }
function echo_green  { echo -e "\033[1;32m$@\033[0m"; }
function echo_blue   { echo -e "\033[1;34m$@\033[0m"; }

EXIT_CODE=0
FAILED_TESTS=()

function print_result() {
  if [ $1 -eq 0 ]; then
    echo_green "[Passed]"
  else
    echo_red "[Failed]"
    EXIT_CODE=1
  fi
  echo ""
}

BASENAME=`basename "$PWD"`
echo $BASENAME
if [ $BASENAME == "script" ]; then
    echo "cd .."
    cd ..
fi

while read p; do
  echo_blue Running Tests `basename "$p"`
  $p
  print_result $? 
  #if [ $EXIT_CODE -ne 0 ]; then
      #FAILED_TESTS+=($p)
      #echo FAILED
  #fi 
done < build/test_list.sh

if [ $EXIT_CODE -eq 0 ]; then
  echo_green "All tests passed!"
else
  echo_red "There were failed tests"
  #for t in "${FAILED_TESTS[@]}"; do echo $t; done
fi

exit $EXIT_CODE


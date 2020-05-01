#!/usr/bin/env bash

function check_diff_py(){
    echo "check_diff_py"
    echo "$1"
    echo "$1" | pycodestyle --diff --max-line-length=1 | echo -e $(cat)
}
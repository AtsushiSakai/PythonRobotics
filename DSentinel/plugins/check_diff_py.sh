#!/usr/bin/env bash

function check_diff_py(){
    echo "check_diff_py"
    git diff --unified=0 $1 -- ${GIT_ROOT_DIR}'/*'.py | pycodestyle --diff | cat
    git diff --unified=0 084c86244798f67fd1fbd29212a9ea36e67e3a97 -- *py | pycodestyle --diff
    echo ${PIPESTATUS[@]}
#    git diff --unified=0 $1 -- ${GIT_ROOT_DIR}'/*'.py | pycodestyle --diff | echo $(cat)
}
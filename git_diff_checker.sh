#!/usr/bin/env bash

# Find when the current branch split off master.
function find_branch_point(){
    echo "find_branch_point"

}

function main(){
    echo "Start git_diff_checker!!"
	# move to this script dir
	cd $(dirname "$0")

    find_branch_point

    echo "Done!!"
}

main "$@"

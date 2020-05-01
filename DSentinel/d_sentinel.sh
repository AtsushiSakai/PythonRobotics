#!/usr/bin/env bash
#
# DSentinel (Diff Sentinel): A simple diff based code check framework
#
# Author: Atsushi Sakai (@Atsushi_twi)
#
set -eo pipefail

PLUGIN_DIR="/plugins/"
CURRENT_DIR=$(pwd)

#######################################
# get commit history
# Globals:
#   None
# Arguments:
#   branch name
#   number of commits
#   commit history (out)
# Outputs:
#   None
#######################################
function get_reversed_commit_history(){
    local branch=$1
    local num_commits=$2
    local -n commit_history=$3
    mapfile -t commit_history < <(git rev-list --max-count ${num_commits} --first-parent ${branch})
}

#######################################
# Search branch commit between current branch and target branch
# Globals:
#   None
# Arguments:
#   current branch name
#   target branch name
#   branch commit hash number (out)
# Outputs:
#   None
#######################################
function search_branch_commit(){
    local current_branch=$1
    local target_branch=$2
    local -n rev=$3
    local current_commits=() target_commits=()
    local MAX_SEARCH_COMMITS=1000
    get_reversed_commit_history ${current_branch} ${MAX_SEARCH_COMMITS} current_commits
    get_reversed_commit_history ${target_branch} ${MAX_SEARCH_COMMITS} target_commits
    for current_commit in "${current_commits[@]}"
    do
        for target_commit in "${target_commits[@]}"
        do
            if [[ "$target_commit" = "$current_commit" ]]; then
                # found same commit hash, which is branch commit
                rev=${current_commit}
                return
            fi
        done
    done
}

#####################################################
# Search diff files from the branch commit
# Globals:
#   None
# Arguments:
#   branch commit of current branch
#   diff file list (out)
# Outputs:
#   None
######################################################
function search_diff_files(){
    local branch_commit=$1
    local -n rev=$2
    # get diff file names
    mapfile -t rev < <(git diff --name-only ${branch_commit})
}

#####################################################
# get diff file extensions from diff file list
# Globals:
#   None
# Arguments:
#   diff file extension list which has no duplication (out)
#   diff file list
# Outputs:
#   None
######################################################
function get_diff_file_extensions(){
    # using hash map to eliminate duplication
    local args=("$@")
    local diff_files=("${args[@]:1}")
    local -n rev=$1
    declare -A hash_map
    for diff_file in "${diff_files[@]}"
    do
        filename=$(basename -- "$diff_file")
        extension="${filename##*.}"
        hash_map[${extension}]=1
    done
    rev=(${!hash_map[@]})
}

function get_diff(){
    local branch_commit=$1
    local extension=$2
    echo ${branch_commit}
    echo ${extension}

    find . -name '*.py'
#    git diff --unified=0 ${branch_commit} -- '***.py'
    git diff -- '*'
    echo $rev

}

#####################################################
# Check diff based on plugin checkers
# Plugin check function name have to be check_diff_<exttention> (ex: check_diff_py)
# Globals:
#   None
# Arguments:
#   diff file list
# Outputs:
#   None
#####################################################
function check_diff(){
    local args=("$@")
    local branch_commit=$1
    local diff_files=("${args[@]:1}")
    local diff_extensions=()
    get_diff_file_extensions diff_extensions ${diff_files[@]}

    for diff_extension in "${diff_extensions[@]}"
    do
        local check_cmd="check_diff_${diff_extension}"
        if type ${check_cmd} > /dev/null 2>&1; then
            echo "command ${check_cmd} found, so lets check these ${diff_extension} diff file"
            get_diff ${branch_commit} ${diff_extension}
            eval "${check_cmd}"
        else
            echo "command ${check_cmd} not found"
        fi
    done
}

#####################################################
# Load all scripts in the plugin directory
# Globals:
#   CURRENT_DIR: current directory
#   PLUGIN_DIR: plugin directory
# Arguments:
#   None
# Outputs:
#   None
#####################################################
function load_plugins(){
    echo "load_plugins"
    for f in ${CURRENT_DIR}${PLUGIN_DIR}*
    do
        echo ${f}" is loaded"
        source ${f}
    done
}

#####################################################
# Main function
# Globals:
#   None
# Arguments:
#   diff extension list
# Outputs:
#   None
#####################################################
function main(){
    echo "Start git_diff_checker!!"
	# move to this script dir
	cd $(dirname "$0")

	local current_branch='HEAD'
	local target_branch='master'

	load_plugins

    search_branch_commit ${current_branch} ${target_branch} branch_commit
    search_diff_files ${branch_commit} diff_files
    check_diff ${branch_commit} ${diff_files[@]}

    echo "Done!!"
}

main "$@"

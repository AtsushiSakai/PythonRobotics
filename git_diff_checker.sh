#!/usr/bin/env bash
#
#
set -eo pipefail

function abort(){
   echo "$@" 1>&2
   exit 1
}

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
# Outputs:
#   branch commit hash number
#######################################
function search_branch_commit(){
    local current_branch=$1
    local target_branch=$2
    local -n rev=$3
    local current_commits=() target_commits=()
    get_reversed_commit_history ${current_branch} 1000 current_commits
    get_reversed_commit_history ${target_branch} 1000 target_commits
    for current_commit in "${current_commits[@]}"
    do
        for target_commit in "${target_commits[@]}"
        do
            if [[ "$target_commit" = "$current_commit" ]]; then
                rev=${current_commit}
                return
            fi
        done
    done
}

function search_diff_files(){
    local branch_commit=$1
    local diff_files=(), diff_extensions=()
    mapfile -t diff_files < <(git diff --name-only ${branch_commit})

    declare -A hm
    for diff_file in "${diff_files[@]}"
    do
        filename=$(basename -- "$diff_file")
        extension="${filename##*.}"
        hm[${extension}]=1
    done
    diff_extensions=(${!hm[@]})

    for diff_extension in "${diff_extension[@]}"
    do
        echo diff_extension
    done
}

function main(){
    echo "Start git_diff_checker!!"
	# move to this script dir
	cd $(dirname "$0")

	local current_branch='HEAD'
	local target_branch='master'

    search_branch_commit ${current_branch} ${target_branch} branch_commit
    search_diff_files ${branch_commit}

    echo "Done!!"
}

main "$@"

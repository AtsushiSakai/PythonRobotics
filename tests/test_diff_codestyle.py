import conftest
import subprocess


def rev_list(branch, num_commits):
    """List commits in reverse chronological order.
    Only the first `num_commits` are shown.
    """
    res = subprocess.run(
        [
            'git',
            'rev-list',
            '--max-count',
            f'{num_commits}',
            '--first-parent',
            branch
        ],
        stdout=subprocess.PIPE,
        encoding='utf-8',
    )
    res.check_returncode()
    return res.stdout.rstrip('\n').split('\n')


def find_branch_point(branch):
    """Find when the current branch split off from the given branch.
    It is based off of this Stackoverflow post:
    https://stackoverflow.com/questions/1527234/finding-a-branch-point-with-git#4991675
    """
    branch_commits = rev_list('HEAD', 1000)
    main_commits = set(rev_list(branch, 1000))
    for branch_commit in branch_commits:
        if branch_commit in main_commits:
            return branch_commit

    # If a branch split off over 1000 commits ago we will fail to find
    # the ancestor.
    raise RuntimeError(
        'Failed to find a common ancestor in the last 1000 commits')


def find_diff(sha):
    """Find the diff since the given sha."""
    files = ['*.py']
    res = subprocess.run(
        ['git', 'diff', '--unified=0', sha, '--'] + files,
        stdout=subprocess.PIPE,
        encoding='utf-8'
    )
    res.check_returncode()
    return res.stdout


def run_flake8(diff):
    """Run flake8 on the given diff."""
    res = subprocess.run(
        ['flake8', '--diff'],
        input=diff,
        stdout=subprocess.PIPE,
        encoding='utf-8',
    )
    return res.returncode, res.stdout


def test():
    branch_commit = find_branch_point("origin/master")
    diff = find_diff(branch_commit)
    rc, errors = run_flake8(diff)
    if errors:
        print(errors)
    else:
        print("No lint errors found.")
    assert rc == 0


if __name__ == '__main__':
    conftest.run_this_test(__file__)

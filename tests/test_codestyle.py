import conftest
import subprocess


def run_flake8(root_dir):
    res = subprocess.run(
        ['flake8', '--ignore',
         'E402',  # top level import for sys.path.append
         root_dir],
        stdout=subprocess.PIPE,
        encoding='utf-8',
    )
    return res.returncode, res.stdout


def test():
    rc, errors = run_flake8("../")
    if errors:
        print(errors)
    else:
        print("No lint errors found.")
    assert rc == 0


if __name__ == '__main__':
    conftest.run_this_test(__file__)

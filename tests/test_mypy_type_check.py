import os
import subprocess

import conftest

SRC_DIR_LIST = [
    "AerialNavigation",
    "ArmNavigation",
    "Bipedal",
    "Control",
    "Localization",
    "Mapping",
    "PathPlanning",
    "PathTracking",
    "SLAM",
]


def run_mypy(dir_name, config_path):
    res = subprocess.run(
        ['mypy',
         '--config-file',
         config_path,
         dir_name],
        stdout=subprocess.PIPE,
        encoding='utf-8', )
    return res.returncode, res.stdout


def test():
    project_dir_path = os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))
    print(f"{project_dir_path=}")

    config_path = os.path.join(project_dir_path, "mypy.ini")
    print(f"{config_path=}")

    for dir_name in SRC_DIR_LIST:
        dir_path = os.path.join(project_dir_path, dir_name)
        print(f"{dir_path=}")
        rc, errors = run_mypy(dir_path, config_path)
        if errors:
            print(errors)
        else:
            print("No lint errors found.")
        assert rc == 0


if __name__ == '__main__':
    conftest.run_this_test(__file__)

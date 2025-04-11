import os
import subprocess

import conftest

SUBPACKAGE_LIST = [
    "AerialNavigation",
    "ArmNavigation",
    "Bipedal",
    "Localization",
    "Mapping",
    "PathPlanning",
    "PathTracking",
    "SLAM",
    "InvertedPendulum"
]


def run_mypy(dir_name, project_path, config_path):
    res = subprocess.run(
        ['mypy',
         '--config-file',
         config_path,
         '-p',
         dir_name],
        cwd=project_path,
        stdout=subprocess.PIPE,
        encoding='utf-8')
    return res.returncode, res.stdout


def test():
    project_dir_path = os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))
    print(f"{project_dir_path=}")

    config_path = os.path.join(project_dir_path, "mypy.ini")
    print(f"{config_path=}")

    for sub_package_name in SUBPACKAGE_LIST:
        rc, errors = run_mypy(sub_package_name, project_dir_path, config_path)
        if errors:
            print(errors)
        else:
            print("No lint errors found.")
        assert rc == 0


if __name__ == '__main__':
    conftest.run_this_test(__file__)


import sys
import os


def add_root_path_to_python_path():
    ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.path.append(ROOT_DIR)

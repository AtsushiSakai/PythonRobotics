""" 

Jupyter notebook converter to rst file 

author: Atsushi Sakai

"""

NOTEBOOK_DIR = "../"

import glob
import os
import os.path
import subprocess


def get_notebook_path_list(ndir):
    path = glob.glob(ndir + "**/*.ipynb", recursive=True)
    return path


def generate_rst(npath):
    # print(npath)

    # generate dir
    dirpath = "./modules/" + os.path.dirname(npath)[3:]
    # print(dirpath)

    if not os.path.exists(dirpath):
        os.makedirs(dirpath)

    rstpath = os.path.abspath("./modules/" + npath[3:-5] + "rst")
    print(rstpath)

    cmd = "jupyter nbconvert --to rst "
    cmd += npath
    cmd += " --output "
    cmd += rstpath
    print(cmd)
    subprocess.call(cmd, shell=True)


def main():
    print("start!!")

    notebook_path_list = get_notebook_path_list(NOTEBOOK_DIR)
    # print(notebook_path_list)

    for npath in notebook_path_list:
        generate_rst(npath)

    print("done!!")


if __name__ == '__main__':
    main()

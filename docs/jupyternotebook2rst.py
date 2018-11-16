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
    dirpath = os.path.dirname(npath)
    # print(dirpath)

    rstpath = os.path.abspath("./modules/" + npath[3:-5] + "rst")
    # print(rstpath)

    basename = os.path.basename(rstpath)

    cmd = "jupyter nbconvert --to rst "
    cmd += npath
    print(cmd)
    subprocess.call(cmd, shell=True)

    cmd = "rm -rf "
    cmd += "./modules/"
    cmd += basename[:-4]
    cmd += "*"
    print(cmd)
    subprocess.call(cmd, shell=True)

    cmd = "mv "
    cmd += dirpath
    cmd += "/*.rst ./modules/"
    print(cmd)
    subprocess.call(cmd, shell=True)

    cmd = "mv "
    cmd += dirpath
    cmd += "/*_files ./modules/"
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

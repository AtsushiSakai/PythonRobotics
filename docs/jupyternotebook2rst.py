"""

Jupyter notebook converter to rst file

author: Atsushi Sakai

"""
import subprocess
import os.path
import os
import glob


NOTEBOOK_DIR = "../"


def get_notebook_path_list(ndir):
    path = glob.glob(ndir + "**/*.ipynb", recursive=True)
    return path


def convert_rst(rstpath):

    with open(rstpath, "r") as bfile:
        filedata = bfile.read()

        # convert from code directive to code-block
        # because showing code in Sphinx
        before = ".. code:: ipython3"
        after = ".. code-block:: ipython3"
        filedata = filedata.replace(before, after)

    with open(rstpath, "w") as afile:
        afile.write(filedata)


def generate_rst(npath):
    print("====Start generating rst======")

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

    rstpath = dirpath + "/" + basename
    convert_rst(rstpath)

    # clean up old files
    cmd = "rm -rf "
    cmd += "./modules/"
    cmd += basename[:-4]
    cmd += "*"
    # print(cmd)
    subprocess.call(cmd, shell=True)

    # move files to module dir
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
        if "template" not in npath:
            generate_rst(npath)

    print("done!!")


if __name__ == '__main__':
    main()

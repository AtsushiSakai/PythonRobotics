"""

Data down loader for pose optimization

author: Atsushi Sakai

"""


import subprocess
def main():
    print("start!!")

    cmd = "wget https://www.dropbox.com/s/zu23p8d522qccor/parking-garage.g2o?dl=0 -O parking-garage.g2o -nc"
    subprocess.call(cmd, shell=True)

    print("done!!")


if __name__ == '__main__':
    main()


"""

Data down loader for pose optimization

author: Atsushi Sakai

"""

import subprocess


def main():
    print("start!!")

    cmd = "wget https://www.dropbox.com/s/zu23p8d522qccor/parking-garage.g2o?dl=0 -O parking-garage.g2o -nc"
    subprocess.call(cmd, shell=True)
    cmd = "wget http://www.furo.org/irie/datasets/torus3d_guess.g2o -nc"
    subprocess.call(cmd, shell=True)
    cmd = "wget http://www.furo.org/irie/datasets/sphere2200_guess.g2o -nc"
    subprocess.call(cmd, shell=True)

    print("done!!")


if __name__ == '__main__':
    main()

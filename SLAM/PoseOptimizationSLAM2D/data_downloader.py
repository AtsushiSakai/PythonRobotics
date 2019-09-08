"""

Data down loader for pose optimization

author: Atsushi Sakai

"""


import subprocess
def main():
    print("start!!")

    cmd = "wget https://www.dropbox.com/s/vcz8cag7bo0zlaj/input_INTEL_g2o.g2o?dl=0 -O intel.g2o -nc"
    subprocess.call(cmd, shell=True)

    cmd = "wget https://www.dropbox.com/s/d8fcn1jg1mebx8f/input_MITb_g2o.g2o?dl=0 -O mit_killian.g2o -nc"

    subprocess.call(cmd, shell=True)
    cmd = "wget https://www.dropbox.com/s/gmdzo74b3tzvbrw/input_M3500_g2o.g2o?dl=0 -O manhattan3500.g2o -nc"
    subprocess.call(cmd, shell=True)

    print("done!!")


if __name__ == '__main__':
    main()


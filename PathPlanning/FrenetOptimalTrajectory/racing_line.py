import sys
from math import sqrt
import numpy as np
import json


class RacingLine:

    def json_parser(json_file):

        race_x = []
        race_y = []
        ins_x = []
        ins_y = []
        out_x = []
        out_y = []

        with open(json_file) as data_file:
            data = json.loads(data_file.read())
            o = data['Outside']
            t = data['Inside']
            p = data['Racing']
            for i, _ in enumerate(t):
                if i%10 == 0:
                    d = t[i]
                    n = d[0]
                    m = d[1]
                    ins_x.append(n)
                    ins_y.append(m)

            for j, _ in enumerate(o):
                if j%10 == 0:
                    c = o[j]
                    x = c[0]
                    y = c[1]
                    out_x.append(x)
                    out_y.append(y)

            for e, _ in enumerate(p):
                if e%10 == 0:
                    z = p[e]
                    g = z[0]
                    h = z[1]
                    race_x.append(g)
                    race_y.append(h)

            # print(len(ins_x))

        return race_x, race_y, ins_x, ins_y, out_x, out_y

    def curved_abscissa(x, y):
        
        c_a = [0] * len(x)

        for i, _ in enumerate(x):
            if(i == 0):
                c_a[0] = 0
            else:     
                c_a[i] = sqrt(pow(x[i] - x[i-1],2) + \
                    pow(y[i] - y[i-1],2)) + c_a[i-1]
        
        # print(round(c_a[-1]))

        return c_a

 

    def print_usage(self):
            print('')

def main(json_file):
    rc = RacingLine
    a = rc.json_parser(json_file)
    rc.curved_abscissa(a[0], a[1])



if __name__ == '__main__':
    if not len(sys.argv) == 2:
        print_usage()
    else:
        main(sys.argv[1])

import sys
from math import sqrt
import numpy as np
import json


class RacingLine:

    def json_parser(self, json_file):

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
                d = t[i]
                n = d[0]
                m = d[1]
                ins_x.append(n)
                ins_y.append(m)

            for j, _ in enumerate(o):
                c = o[j]
                x = c[0]
                y = c[1]
                out_x.append(x)
                out_y.append(y)

            for e, _ in enumerate(p):
                if e%20 != 0:
                    z = p[e]
                    g = z[0]
                    h = z[1]
                    race_x.append(g)
                    race_y.append(h)

        return race_x, race_y, ins_x, ins_y, out_x, out_y

    def curved_abscissa(self):
        
        c_a = [0] * len(self.race_x)

        for i in range(len(self.race_x)):
            if(i == 0):
                c_a[0] = 0
            else:     
                c_a[i] = sqrt(pow(self.race_x[i] - self.race_x[i-1],2)+pow(self.race_y[i] - self.race_y[i-1],2)) + c_a[i-1]

        return c_a

 

    def print_usage(self):
            print('')

def main(json_file):
    rc = RacingLine()
    rc.json_parser(json_file)
    rc.curved_abscissa()



if __name__ == '__main__':
    if not len(sys.argv) == 2:
        print_usage()
    else:
        main(sys.argv[1])

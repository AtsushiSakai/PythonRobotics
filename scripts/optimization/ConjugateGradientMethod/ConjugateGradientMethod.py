#!/usr/bin/python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
import random
import math

delta = 0.1
minXY=-5.0
maxXY=5.0
nContour=50
alpha=0.001

def Jacob(state):
    u"""
    jacobi matrix of Himmelblau's function
    """
    x=state[0]
    y=state[1]
    dx=4*x**3+4*x*y-44*x+2*x+2*y**2-14
    dy=2*x**2+4*x*y+4*y**3-26*y-22
    J=np.array([dx,dy])
    return J

def HimmelblauFunction(x,y):
    u"""
    Himmelblau's function
    see Himmelblau's function - Wikipedia, the free encyclopedia 
    http://en.wikipedia.org/wiki/Himmelblau%27s_function
    """
    return (x**2+y-11)**2+(x+y**2-7)**2

def CreateMeshData():
    x = np.arange(minXY, maxXY, delta)
    y = np.arange(minXY, maxXY, delta)
    X, Y = np.meshgrid(x, y)
    Z=[HimmelblauFunction(x,y) for (x,y) in zip(X,Y)]
    return(X,Y,Z)

def ConjugateGradientMethod(start,Jacob):
    u"""
    Conjugate Gradient Method Optimization
    """

    result=start
    x=start
    preJ=None

    while 1:
        J=Jacob(x)

        #convergence check
        sumJ=sum([abs(alpha*j) for j in J])
        if sumJ<=0.01:
            print("OK")
            break

        if preJ is not None:
            beta=np.linalg.norm(J)**2/np.linalg.norm(preJ)**2
            grad=-1.0*J+beta*grad

        else:
            grad=-1.0*J
            
        x=x+[alpha*g for g in grad]
        result=np.vstack((result,x))
        #  print(x)

        if math.isnan(x[0]):
            print("nan")
            break
        

        preJ=-1.0*J


    return result

# Main
start=np.array([random.uniform(minXY,maxXY),random.uniform(minXY,maxXY)])

result=ConjugateGradientMethod(start,Jacob)
(X,Y,Z)=CreateMeshData()
CS = plt.contour(X, Y, Z,nContour)
#  plt.clabel(CS, inline=1, fontsize=10)
#  plt.title('Simplest default with labels')

plt.plot(start[0],start[1],"xr");

optX=[x[0] for x in result]
optY=[x[1] for x in result]
plt.plot(optX,optY,"-r");

plt.show()


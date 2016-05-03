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
    x=state[0,0]
    y=state[0,1]
    dx=4*x**3+4*x*y-44*x+2*x+2*y**2-14
    dy=2*x**2+4*x*y+4*y**3-26*y-22
    J=np.matrix([dx,dy]).T
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

def QuasiNewtonMethod(start,Jacob):
    u"""
    Quasi Newton Method Optimization
    """

    result=start
    x=start

    H= np.identity(2)
    preJ=None
    preG=None

    while 1:
        J=Jacob(x)

        sumJ=abs(np.sum(J))
        if sumJ<=0.01:
            print("OK")
            break

        grad=-np.linalg.inv(H)*J
        x+=alpha*grad.T
        
        result=np.vstack((result,np.array(x)))

        if preJ is not None:
            y=J-preJ
            H=H+(y*y.T)/(y.T*preG)-(H*preG*preG.T*H)/(preG.T*H*preG)

        preJ=J
        preG=(alpha*grad.T).T

    return result

# Main
start=np.matrix([random.uniform(minXY,maxXY),random.uniform(minXY,maxXY)])

result=QuasiNewtonMethod(start,Jacob)
(X,Y,Z)=CreateMeshData()
CS = plt.contour(X, Y, Z,nContour)

plt.plot(start[0,0],start[0,1],"xr");

optX=result[:,0]
optY=result[:,1]
plt.plot(optX,optY,"-r");

plt.show()


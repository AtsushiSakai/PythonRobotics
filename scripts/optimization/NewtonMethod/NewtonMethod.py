#!/usr/bin/python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
import random

delta = 0.1
minXY=-5.0
maxXY=5.0
nContour=50
alpha=0.01

def Hessian(state):
    u"""
    Hessian matrix of Himmelblau's function
    """
    x=state[0]
    y=state[1]
    dxx=12*x**2+4*y-42;
    dxy=4*x+4*y
    dyy=4*x+12*y**2-26
    H=np.array([[dxx,dxy],[dxy,dyy]])
    return H
    

def Jacob(state):
    u"""
    jacobi matrix of Himmelblau's function
    """
    x=state[0]
    y=state[1]
    dx=4*x**3+4*x*y-44*x+2*x+2*y**2-14
    dy=2*x**2+4*x*y+4*y**3-26*y-22
    J=[dx,dy]
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

def NewtonMethod(start,Jacob):
    u"""
    Newton Method Optimization
    """

    result=start
    x=start

    while 1:
        J=Jacob(x)
        H=Hessian(x)
        sumJ=sum([abs(alpha*j) for j in J])
        if sumJ<=0.01:
            print("OK")
            break

        grad=-np.linalg.inv(H).dot(J) 
        print(grad)

        x=x+[alpha*j for j in grad]
        
        result=np.vstack((result,x))

    return result

# Main
start=np.array([random.uniform(minXY,maxXY),random.uniform(minXY,maxXY)])

result=NewtonMethod(start,Jacob)
(X,Y,Z)=CreateMeshData()
CS = plt.contour(X, Y, Z,nContour)
#  plt.clabel(CS, inline=1, fontsize=10)
#  plt.title('Simplest default with labels')

plt.plot(start[0],start[1],"xr");

optX=[x[0] for x in result]
optY=[x[1] for x in result]
plt.plot(optX,optY,"-r");

plt.show()


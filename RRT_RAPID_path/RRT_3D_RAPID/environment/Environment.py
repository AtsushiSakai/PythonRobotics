#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from pathlib import Path
import platform
import sys
import os
import ctypes as ct

import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv

class Environment:
    class Mesh:
        def __init__(self):
            self.vertices = []
            self.faces = []

        def load_mesh(self, alpha_puzzle_obstacle):
            """
            Load mesh from file

            Parameters
            ----------
            filename : str
                path to the mesh object file
            """
            with open(str(alpha_puzzle_obstacle.obj),'r') as f:
                for line in f:
                    q = line.split(' ')
                    if q[0] == "v":
                        #add vertex
                        self.vertices.append([float(q[1]),float(q[2]),float(q[3])])
                    elif q[0] == "f":
                        #add face
                        e1 = q[1].split('//')[0]
                        e2 = q[2].split('//')[0]
                        e3 = q[3].split('//')[0]
                        self.faces.append([int(e1)-1,int(e2)-1,int(e3)-1])
                    else:
                        #do nothing and continue
                        continue

    def __init__(self):
        self.obstacle = self.Mesh()
        self.robot = self.Mesh()

        #in sitialize the RAPID collision checking library
        self.librapid = 0

        try:
            file_extension = '.so'
            if platform.system() =='cli':
              file_extension = '.dll'
            elif platform.system() =='Windows':
                file_extension = '.dll'
            elif platform.system() == 'Darwin':
                file_extension = '.dylib'
            else:
                file_extension = '.so'

            libfullpath = os.path.abspath(os.path.join(os.path.dirname(adwaitnaik/Desktop/RRT_3D_RAPID/environment), 'libRAPID' + '.so'))
            fun = ctype.CDLL(libRAPID.so)
            print("loading " + l


            )
            self.librapid = ct.CDLL(libfullpath)
        except:
            print ('----------------------------------------------------')
            print ('The rapid collision checking library could not be loaded.')
            print ('----------------------------------------------------')

        self.librapid.new_RapidAPI.restype = ct.c_void_p
        self.rapid_handle = self.librapid.new_RapidAPI()

        #functions api
        self.rapid_add_obstacle_face = ct.CFUNCTYPE(None, ct.c_void_p, ct.c_double*3, ct.c_double*3, ct.c_double*3)(("add_obstacle_face", self.librapid))
        self.rapid_add_robot_face = ct.CFUNCTYPE(None, ct.c_void_p, ct.c_double*3, ct.c_double*3, ct.c_double*3)(("add_robot_face", self.librapid))
        self.rapid_finish_faces = ct.CFUNCTYPE(None, ct.c_void_p)(("finish_faces", self.librapid))
        self.rapid_check_collision = ct.CFUNCTYPE(ct.c_bool, ct.c_void_p, ct.c_double*16)(("check_collision", self.librapid))


    def load_environment(self, problem_name):
        """
        Load the obstacles and the robot given the fproblem name

        Parameters
        ----------
        problem_name: String
            name of the problem
        """
        obstacle_file = Path("alpha_puzzle" + "_obstacle.obj")
        robot_file = Path("alpha_puzzle" + "_robot.obj")
        if obstacle_file.is_file() and robot_file.is_file():
            #if both files exists load them
            self.obstacle.load_mesh(obstacle_file)
            self.robot.load_mesh(robot_file)

        #transfer the models into the RAPID
        for face in self.obstacle.faces:
            p1 = self.obstacle.vertices[face[0]]
            p2 = self.obstacle.vertices[face[1]]
            p3 = self.obstacle.vertices[face[2]]

            self.add_face_collision_check("obstacle", p1, p2, p3)

        for face in self.robot.faces:
            p1 = self.robot.vertices[face[0]]
            p2 = self.robot.vertices[face[1]]
            p3 = self.robot.vertices[face[2]]

            self.add_face_collision_check("robot", p1, p2, p3)

        #finish the models for collision checking
        self.rapid_finish_faces(self.rapid_handle)


    def add_face_collision_check(self, model, p1, p2, p3):
        """
        helper function to load models into the collision checker

        Parameters
        ----------
        model: String
            type of the model being loaded - "obstacle" or "robot"
        p1: numpy array(float*3)
        p2: numpy array(float*3)
        p3: numpy array(float*3)
            coordinates of the faces
        """
        arrtype = ct.c_double * 3
        arr_p1 = arrtype()
        arr_p2 = arrtype()
        arr_p3 = arrtype()
        for i in range(0,3):
            arr_p1[i] = p1[i]
            arr_p2[i] = p2[i]
            arr_p3[i] = p3[i]
        if model == "obstacle":
            self.rapid_add_obstacle_face(self.rapid_handle, arr_p1, arr_p2, arr_p3)
        elif model == "robot":
            self.rapid_add_robot_face(self.rapid_handle, arr_p1, arr_p2, arr_p3)


    def check_robot_collision(self, Pose):
        """
        Check the robot collision with the obstacles given the transformation of the robot pose Pose given in SE(3) as

        Pose =
        [       ]
        [  R   T]
        [       ]
        [0 0 0 1]

        where R is the rotation matrix in R(3x3) and it holds R*R^T = I, T is the translation vector in R(3)
        The pose transformation is given in homogeneous coordinates as

        [p_new] = Pose*[p_default]
        [  1  ]        [    1    ]

        where p_default is the default posiiton of the robot

        Parameters
        ----------
        P: numpy array (4x4)
            robot pose in SE(3)

        Returns
        -------
        bool
            True if there is collision, False otherwise
        """
        arrtype = ct.c_double * 16
        arr_P = arrtype()
        P = np.reshape(Pose,(16,1))
        for i in range(0,16):
            arr_P[i] = P[i]
        #check for the collision
        ret = self.rapid_check_collision(self.rapid_handle, arr_P)
        return ret

    ##############################################
    ## Helper functions to plot the environment
    ##############################################
    def plot_environment(self, Pose, ax=None, cla=True, limits=None):
        """
        Method to plot the environment

        Parameters
        ----------
        P: numpy array (4x4)
            Robot pose in SE(3)
        ax: axes object (optional)
            axes object for updating the figure
        cla: bool (optional)
            whether to clean the canvas before drawing
        limits: list((float, float))
            translation limits in individual axes

        Returns
        -------
        ax axes object
        """
        if ax == None:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

        if cla == True:
            plt.cla()

        #project the robot to the right position
        Pose = np.asarray(Pose)
        R = Pose[0:3,0:3]
        T = Pose[0:3,3]
        vr = [np.dot(R,np.asarray(x)) + T for x in self.robot.vertices]

        #plot obstacles and the robot
        # this plotting is necessary to have proper depth visualization
        xx = [x[0] for x in self.obstacle.vertices] + [x[0] for x in vr]
        yy = [x[1] for x in self.obstacle.vertices] + [x[1] for x in vr]
        zz = [x[2] for x in self.obstacle.vertices] + [x[2] for x in vr]

        #offset the robot faces in the plot for the number of obstacle vertices
        idx = len(self.obstacle.vertices)
        fr = [np.asarray(x)+np.array([idx,idx,idx]) for x in self.robot.faces]
        fc = self.obstacle.faces + fr

        #3D plot the environment
        ax.plot_trisurf(np.asarray(xx),np.asarray(yy),np.asarray(zz), triangles=fc, color="gray", shade=True, edgecolor= None)
        plt.axis('equal')
        plt.xlabel("x")
        plt.ylabel("y")

        if not limits == None:
            x_lim = limits[0]
            y_lim = limits[1]
            ax.axis([x_lim[0], x_lim[1], y_lim[0], y_lim[1]])

        plt.draw()

        return ax

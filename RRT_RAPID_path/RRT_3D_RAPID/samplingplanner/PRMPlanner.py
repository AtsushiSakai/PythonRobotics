#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
import numpy as np

import collections
import heapq

import matplotlib.pyplot as plt

import Environment as env

class PRMPlanner:

    def __init__(self, dof, limits):
        """
        Parameters
        ----------
        dof: int
            number of dimensions to solve the problem
            2 .. only x and y 
            3 .. x,y and yaw 
            6 .. x,y,z, yaw,pitch,roll
        limits: list((float, float))
            translation limits in individual axes 
        """
        self.dof = dof
        self.limits = limits

    def plan(self, environment, start, goal):
        """
        Method to plan the path

        Parameters
        ----------
        environment: Environment
            Map of the environment that provides collision checking 
        start: numpy array (4x4)
            start pose of the robot given in SE(3)
        goal: numpy array (4x4)
            goal pose of the robot given in SE(3)

        Returns
        -------
        list(numpy array (4x4))
            the path between the start and the goal Pose in SE(3) coordinates
        """
        #TODO: Task05 Implement the Probabilistic roadmap algorithm

        path = []
        path.append(start)
        
        #get 20 interlying points between start and goal and add them to the path
        n_points = 20
        T_start = start[0:3,3]
        T_goal = goal[0:3,3]
        T_dir = (T_goal-T_start)
            
        yaw = 0
        for i in range(0,n_points):

            if self.dof == 2:
                R = np.eye(3)
            else:
                #provide the heading using euler angles (yaw, pitch, roll) - note, in 2D use only yaw
                #rotate the robot a little bit each step
                yaw += math.pi/10
                pitch = 0
                roll = 0
                #construct the rotation matrix
                R = self.rotation_matrix(yaw, pitch, roll) 

            #get the interlying point
            T = T_start + T_dir*i/n_points

            #construct the SE(3) matrix
            P = np.hstack((R,T.reshape((3,1))))
            P = np.vstack((P,[0,0,0,1]))

            #check for collision with the environment
            ret = environment.check_robot_collision(P)

            #if not in collision add to path
            if ret == False:
                #add it to path
                path.append(P)

        return(path)

    
    def rotation_matrix(self, yaw, pitch, roll):
        """
        Constructs rotation matrix given the euler angles

        Parameters
        ----------
        yaw: float
        pitch: float
        roll: float
            respective euler angles
        """
        R_x = np.array([[1, 0, 0],
                        [0, math.cos(roll), math.sin(roll)],
                        [0, -math.sin(roll), math.cos(roll)]])
        R_y = np.array([[math.cos(pitch), 0, -math.sin(pitch)],
                        [0, 1, 0],
                        [math.sin(pitch), 0, math.cos(pitch)]])
        R_z = np.array([[math.cos(yaw), math.sin(yaw), 0],
                        [-math.sin(yaw), math.cos(yaw), 0],
                        [0, 0, 1]])

        R = R_x.dot(R_y).dot(R_z)
        return R






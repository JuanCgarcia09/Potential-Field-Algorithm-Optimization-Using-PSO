#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 26 11:25:35 2019

@author: jacobadkins1
"""

import numpy as np
import matplotlib.pyplot as mlp
import sys
import sim as vrep # access all the VREP elements
#import vrep
import time

vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # start aconnection
if clientID!=-1:
    print ("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")    
    
    
# get object handles for drone target and sensors
returnCode,quad_body = vrep.simxGetObjectHandle(clientID,"Quadricopter_base",vrep.simx_opmode_blocking)
returnCode,quad_target = vrep.simxGetObjectHandle(clientID,"Quadricopter_target",vrep.simx_opmode_blocking)
returnCode,goal = vrep.simxGetObjectHandle(clientID,"Cuboid3",vrep.simx_opmode_blocking)



returnCode,front_sensor = vrep.simxGetObjectHandle(clientID,"Quad_front_sensor",vrep.simx_opmode_blocking)


# proximity readings from sensors
returnCode,positionDrone=vrep.simxGetObjectPosition(clientID, quad_body,-1,vrep.simx_opmode_streaming)
returnCode,positionGoal=vrep.simxGetObjectPosition(clientID, goal,-1,vrep.simx_opmode_streaming)
time.sleep(3)
# front sensor
returnCode,detectionStateF,frontM,x2,x1 = vrep.simxReadProximitySensor(clientID,front_sensor,vrep.simx_opmode_streaming);
frontM=np.asarray(frontM)
    
time.sleep(2)

m=0
Kp=1


for i in range(0,1000):
    returnCode,positionDrone=vrep.simxGetObjectPosition(clientID, quad_body,-1,vrep.simx_opmode_streaming)
    positionDrone=np.asarray(positionDrone)
    returnCode = vrep.simxSetObjectPosition(clientID, quad_target,  -1, (positionDrone[0]+0.5,  positionDrone[1]+0.5, 2),vrep.simx_opmode_oneshot)
    time.sleep(0.2)
    





#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 26 11:25:35 2019

@author: jacobadkins1
"""

import numpy as np

import sys
import sim as vrep # access all the VREP elements
#import vrep
import time
import matplotlib.pyplot as plt
import math

class Particula(object):
    
    def __init__(self,x,v):
        self.posicion=x
        self.velocidad=v
        self.mejorValor=10000
        self.mejorPosicion=x
        self.valor=10000
        
    def evaluar(self, funcion, inputs,targets):
        valor=funcion(self.posicion,inputs, targets)
        if valor<self.mejorValor:
            self.mejorValor=valor
            self.mejorPosicion=self.posicion
        self.valor=valor
                    
    def actualizarVel(self,mejorPosEnj, C):
        for i in range(len(mejorPosEnj)):
            self.velocidad[i]= C[0]*self.velocidad[i] + C[1]*(self.mejorPosicion[i]-self.posicion[i]) + C[2]*(mejorPosEnj[i]-self.posicion[i])

    def actualizarPos(self,dt):
        self.posicion=self.posicion+ np.array( self.velocidad)  *dt
#  evaluar particula
        
#  actualizar vel
#  actualizar pos        


def funEval(posicion, inputs, targets):
    errorCode, PosQuadtarget  =vrep.simxGetObjectPosition(clientID,quad_target,-1,vrep.simx_opmode_blocking)
    vrep.simxSetObjectPosition(clientID,quad_target ,  -1, (Posini[0],Posini[1],PosQuadtarget[2]),vrep.simx_opmode_oneshot)
    vrep.simxSetObjectPosition(clientID,quad_body ,  -1, (Posini[0],Posini[1],PosQuadtarget[2]),vrep.simx_opmode_oneshot)
    inicio=time.time()
    kpObs=posicion[0]
    KpGoal=posicion[1]
    Timbrebipbip=True
    while Timbrebipbip: 
        return_code, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = vrep.simxReadProximitySensor(clientID,  front_sensor , vrep.simx_opmode_oneshot_wait)
        
        errorCode, PosQuadtarget  =vrep.simxGetObjectPosition(clientID,quad_target,-1,vrep.simx_opmode_blocking)
        errorCode, PosGoal  =vrep.simxGetObjectPosition(clientID,goal,-1,vrep.simx_opmode_blocking)
        errorCode, PosObj  =vrep.simxGetObjectPosition(clientID,detected_object_handle,-1,vrep.simx_opmode_blocking)
        
        VxObs,VyObs=kpObs*((PosObj[0]-PosQuadtarget[0])/2.4),kpObs*((PosObj[1]-PosQuadtarget[1])/2.4)
        Vxgoal,Vygoal=KpGoal*((PosGoal[0]-PosQuadtarget[0])/20),KpGoal*((PosGoal[1]-PosQuadtarget[1])/20)
        if(abs(Vxgoal)+abs(Vygoal)<0.5):
            Vxgoal=Vxgoal*1.5
            Vygoal=Vygoal*1.5
            if(abs(Vxgoal)+abs(Vygoal)<0.25):
                Vxgoal=Vxgoal*3
                Vygoal=Vygoal*3
        anguloobs=math.degrees(math.atan((PosObj[1]-PosQuadtarget[1])/(PosObj[0]-PosQuadtarget[0])))
        
        if(PosObj[0]<PosQuadtarget[0] and PosObj[1]<PosQuadtarget[1]):
            anguloobs=anguloobs-180
        elif(PosObj[0]<PosQuadtarget[0] and PosObj[1]>PosQuadtarget[1]):
            anguloobs=anguloobs+180
        
        angulogoal=math.degrees(math.atan((PosGoal[1]-PosQuadtarget[1])/(PosGoal[0]-PosQuadtarget[0])))
        
        if(PosGoal[0]<PosQuadtarget[0] and PosGoal[1]<PosQuadtarget[1]):
            angulogoal=angulogoal-180
        elif(PosGoal[0]<PosQuadtarget[0] and PosGoal[1]>PosQuadtarget[1]):
            angulogoal=angulogoal+180
        dif=anguloobs-angulogoal
        if(dif>-5 and dif<5):
            VxObs=+0.1
        
        
        if(detection_state==False):
            VxObs=0
            VyObs=0 
        
        vrep.simxSetObjectPosition(clientID, quad_target,  -1, (PosQuadtarget[0]-VxObs+Vxgoal,  PosQuadtarget[1]+Vygoal-VyObs, PosQuadtarget[2]),vrep.simx_opmode_oneshot)
        if((PosQuadtarget[0]-PosGoal[0])**2+(PosQuadtarget[1]-PosGoal[1])**2<0.15):
            Timbrebipbip=False
    fin=time.time()
    tiempo=fin-inicio
    error=abs(tiempo-targets)
    return error 




vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # start aconnection
if clientID!=-1:
    print ("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")    
    
    
    
    

# get object handles for drone target and sensors
returnCode,quad_body = vrep.simxGetObjectHandle(clientID,"Quadricopter",vrep.simx_opmode_blocking)
returnCode,quad_target = vrep.simxGetObjectHandle(clientID,"Quadricopter_target",vrep.simx_opmode_blocking)
returnCode,goal = vrep.simxGetObjectHandle(clientID,"Cuboid3",vrep.simx_opmode_blocking)
returnCode,ini = vrep.simxGetObjectHandle(clientID,"Cuboid4",vrep.simx_opmode_blocking)


returnCode,front_sensor = vrep.simxGetObjectHandle(clientID,"Quad_front_sensor",vrep.simx_opmode_blocking)
errorCode, Posini  =vrep.simxGetObjectPosition(clientID,ini,-1,vrep.simx_opmode_blocking)
"""
# proximity readings from sensors
returnCode,positionDrone=vrep.simxGetObjectPosition(clientID, quad_body,-1,vrep.simx_opmode_streaming)
returnCode,positionGoal=vrep.simxGetObjectPosition(clientID, goal,-1,vrep.simx_opmode_streaming)
returnCode,positionini=vrep.simxGetObjectPosition(clientID, ini,-1,vrep.simx_opmode_streaming)
time.sleep(3)
# front sensor
return_code, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = vrep.simxReadProximitySensor(clientID,  front_sensor , vrep.simx_opmode_oneshot_wait)
    
time.sleep(2)


returnCode,positionDrone=vrep.simxGetObjectPosition(clientID, quad_body,-1,vrep.simx_opmode_streaming)
positionDrone=np.asarray(positionDrone)
"""

N=1 # numero de particulas
dt=0.01 # Tamanio del salto
# limites del universo de busqueda
KpObs_min=0.25
KpObs_max=0.3
KpGoal_min=0.2
KpGoal_max=0.3

epocas=15
C=[0.2, 0.5, 0.9]
particulas=[]
errorHist=np.empty([epocas])
posHist=np.empty([epocas,2])
posHistPar=np.empty([epocas,N,2])
for i in range(N):
    KpObs_temp=np.random.random(1)*(KpObs_max-KpObs_min)+KpObs_min
    KpGoal_temp=np.random.random(1)*(KpGoal_max-KpGoal_min)+KpGoal_min

    
    particulas.append(Particula( [KpObs_temp,KpGoal_temp], [0,0]))
    

for i in range(epocas):
    valores=[]
    for particula in particulas:
        particula.evaluar(funEval, 0,5)
        valores.append(particula.valor)
    
    for particula in particulas:
        mejorValEnj=min(valores)
        mejorPosEnj=particulas[valores.index(min(valores))].posicion
        particula.actualizarVel(mejorPosEnj, C)
        particula.actualizarPos(dt)

    print('Epoca: ', i)
    print('Velocidad ', particula.velocidad)
    print('posicion ', particula.posicion)
    print('Mejorvalor ', particula.mejorValor)
    print('Valor ', particula.valor)
    errorHist[i]=mejorValEnj
    for j in range(N):
        posHistPar[i,j,:]=particulas[j].posicion.T

print('posicion:  ', mejorPosEnj)



errorHist=errorHist+5
plt.figure(1, clear=True)
plt.plot(errorHist,'g',label="Tiempo vs épocas")
plt.grid()
plt.legend()
plt.xlabel('Épocas')
plt.ylabel('Tiempo')


    
#returnCode = vrep.simxSetObjectPosition(clientID, quad_target,  -1, (positionDrone[0]+0.5,  positionDrone[1]+0, 2),vrep.simx_opmode_oneshot)
#vrep.simxSetObjectPosition(clientID, goal,  -1, (positionDrone[0]+1,  positionDrone[1]+1),vrep.simx_opmode_oneshot)
#vrep.simxSetObjectPosition(clientID,quad_target ,  -1, (-6,  8,2.20181847),vrep.simx_opmode_oneshot)
#vrep.simxSetObjectPosition(clientID,quad_body ,  -1, (-6,  8,2.20181847),vrep.simx_opmode_oneshot)




from controller import Robot, Motor, Receiver, Emitter, TouchSensor

import numpy as np
import random
import math
import struct
from matplotlib import pyplot as plt

robot = Robot()
timestep = int(robot.getBasicTimeStep())


EPUCK_AXLE_DIAMETER  = 0.053 
MAX_SPEED = 6.28
MAX_SPEED_MS = .125

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

receiver = robot.getDevice("receiver")
receiver.enable(timestep)

emitter = robot.getDevice("emitter")

pose_x = 0
pose_y = 0
pose_theta = 0

vL = 0 
vR = 0 


gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice('gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(timestep)




####################################

territorial = True
while robot.step(timestep) != -1  and territorial:
    if receiver.getQueueLength() > 0:
        receivedData = receiver.getData()
        tup = struct.unpack('c', receivedData)
        if tup[0].decode("utf-8") == 'E':
            print("Detected Bot In Ring!")
            territorial = False
            vR = 0
            vL = 0
            leftMotor.setVelocity(vL)
            rightMotor.setVelocity(vR)
        receiver.nextPacket()
        break;
        
        
    vR = MAX_SPEED     
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
    

fightStart = False
xr = 0
theta_r = 0
target_x = .003
target_y = .15
target_theta = 1.8

spinning = False
#territorial robot fight mode
while robot.step(timestep) != -1  and (not territorial):
    
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()
       
    p = math.sqrt(pow((pose_x - target_x),2) + pow((pose_y - target_y),2)) #pos
    a = math.atan2((target_y - pose_y),(target_x - pose_x)) - pose_theta #bearing
    n = target_theta - pose_theta
    
    if(p > .001): #if hasnt approached target yet
        theta_r = 10 * a 
    
         
    if(a < .0000001 and a > -.0000001): 
        xr = p
    else: 
        xr = 0 #stop
       
    
    vL = ((2 * xr) - (theta_r * EPUCK_AXLE_DIAMETER)) / 2
    vR = ((2 * xr) + (theta_r * EPUCK_AXLE_DIAMETER)) / 2
   
    vL = MAX_SPEED * vL
    vR = MAX_SPEED * vR
   
    if(vR > MAX_SPEED):
        vR = MAX_SPEED
    if(vR < -MAX_SPEED): #pos and negative clamp for vR
        vR = -MAX_SPEED
    if(vL > MAX_SPEED):
        vL = MAX_SPEED
    if(vL < -MAX_SPEED): #pos and negative clamp for vL
        vL = -MAX_SPEED

    if(p < .1): #stopping criteria, if goal is reached
       print("territorial spinning commence")
       spinning = True
       break
       
    
    distL = vL/MAX_SPEED * MAX_SPEED_MS * timestep/1000.0
    distR = vR/MAX_SPEED * MAX_SPEED_MS * timestep/1000.0
    pose_x += (distL+distR) / 2.0 * math.cos(pose_theta)
    pose_y += (distL+distR) / 2.0 * math.sin(pose_theta)
    pose_theta += (distR-distL)/EPUCK_AXLE_DIAMETER
    
    if pose_theta > 6.28+3.14/2: pose_theta -= 6.28
    if pose_theta < -3.14: pose_theta += 6.28
    
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
    
health = 3

#contraints to make sure they emit start messages at the same time as to not allow an unfair advantage
otherReady = False
msgSent = False       

while robot.step(timestep) != -1 and spinning:              
    #checking if health is 0
    if(health == 0):
        message = struct.pack('c', 'D'.encode()) #sending out a death notif
        emitter.send(message)
        #dying
        vL = 0
        vR = 0
        leftMotor.setVelocity(vL)
        rightMotor.setVelocity(vR)
        print("territorial bot has died. wanderer bot is the winner")
        break    
    
    
    #spinning
    vL = MAX_SPEED
    vR = 0
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
    
    #sending the initial begin message
    if(not otherReady) and (not msgSent):
        message = struct.pack('c', 'B'.encode()) #sending out ready notif
        emitter.send(message)
        msgSent = True
    
    #checking queue for the begin message 
    if receiver.getQueueLength() > 0 and not otherReady:
        receivedData = receiver.getData()
        tup = struct.unpack('c', receivedData)
        if tup[0].decode("utf-8") == 'B':
            otherReady = True
            
        receiver.nextPacket()
    
    #sends out punch every while loop iteration    
    if(otherReady):
        message = struct.pack('c', 'P'.encode()) #sending out a punch
        emitter.send(message) 
    
    #random number to determine if the punch hits 
    rng = random.randrange(0,100)  
    
    #checks message queue for punches or death notifications  
    if receiver.getQueueLength() > 0 and otherReady:
        receivedData = receiver.getData()
        tup = struct.unpack('c', receivedData)
        #print("territorial getting punches" , tup[0])
        if tup[0].decode("utf-8") == 'P': 
            if (rng == 99):
                health -= 1
                print("territorial bot has recieved one puch, health is now ", health, "/3")
        
        if tup[0].decode("utf-8") == 'D':
            vL = 0
            vR = 0
            leftMotor.setVelocity(vL)
            rightMotor.setVelocity(vR)
        
        receiver.nextPacket()

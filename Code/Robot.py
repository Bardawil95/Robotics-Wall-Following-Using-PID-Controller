# Make sure to have the server side running in V-REP: 
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time, math, random, matplotlib.pyplot as plt


class Robot():

    def __init__(self):

        # Setup Motors
        res, self.leftMotor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking)
        res, self.rightMotor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking)

        # Setup Sonars
        res, self.frontSonar = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor5',sim.simx_opmode_blocking)
        res, self.leftSonar = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor1',sim.simx_opmode_blocking)
        res, self.rightSonar = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor8',sim.simx_opmode_blocking)
        res, self.rightFrontSonar = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor7',sim.simx_opmode_blocking)

        # Start Sonars
        res,detectionStateF,detectedPointF,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID,self.frontSonar,sim.simx_opmode_streaming)
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID,self.leftSonar,sim.simx_opmode_streaming)
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID,self.rightSonar,sim.simx_opmode_streaming)
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID,self.rightFrontSonar,sim.simx_opmode_streaming)
        time.sleep(2)


    def getDistanceReading(self, objectHandle):
        # Get reading from sensor
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID,objectHandle,sim.simx_opmode_buffer)

        if detectionState == 1:
            # return magnitude of detectedPoint
            return math.sqrt(sum(i**2 for i in detectedPoint))
        else:
            # return another value that we know cannot be true and handle it (use a large number so that if you do 'distance < reading' it will work)
            return 9999

    def sonar(self, son):
        if son == 1:
            return self.getDistanceReading(self.frontSonar) 
        elif son == 2: 
            return self.getDistanceReading(self.rightSonar)


    def stop(self):
        res = sim.simxSetJointTargetVelocity(clientID, self.leftMotor, 0, sim.simx_opmode_blocking)
        res = sim.simxSetJointTargetVelocity(clientID, self.rightMotor, 0, sim.simx_opmode_blocking)

    def turnArc(self, leftMotorVelocity, rightMotorVelocity):
        res = sim.simxSetJointTargetVelocity(clientID, self.leftMotor, leftMotorVelocity, sim.simx_opmode_blocking)
        res = sim.simxSetJointTargetVelocity(clientID, self.rightMotor, rightMotorVelocity, sim.simx_opmode_blocking)

    def position(self):
        res, pos1 = sim.simxGetObjectPosition(clientID, self.leftMotor, -1, sim.simx_opmode_blocking)
        res, pos2 = sim.simxGetObjectPosition(clientID, self.rightMotor, -1, sim.simx_opmode_blocking)
        return pos1, pos2

    #Calculate PID
    def PID(self, current_error, previous_error, arr):        
        dt = 0.01
        sum_error = 0
        sum_error = (previous_error + current_error)*dt    
        current_error_derivative = (current_error - previous_error)/dt
        PID = arr["Kp"] * current_error + arr["Ki"] * sum_error + arr["Kd"] * current_error_derivative
        right_PID = 1 + PID
        left_PID = 1 - PID
        return (PID, right_PID, left_PID)
        

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP

curr_err_arr = []
PID_arr = []
if clientID!=-1:
    print ('Connected to remote API server')
    robot = Robot()
    previous_error = 0

    while 1:
        right_sonar_val = robot.sonar(2)
        if right_sonar_val == 9999:
            right_sonar_val = 0
        current_error = right_sonar_val - 0.5   #0.6 is the reference distance value (threshold)

        PID, right_PID, left_PID = robot.PID(current_error, previous_error, {"Kp":0.2, "Ki":0.006, "Kd":0.005})

        # print("Current Error: ", current_error)

        if robot.sonar(1) != 9999 or robot.getDistanceReading(robot.rightFrontSonar) != 9999:      # threshold value
            robot.turnArc(0, 2.5)
            # print(right_PID/2)
        else:                                                       #near right wall
            robot.turnArc(left_PID, right_PID)
            Pos1, Pos2 = robot.position()
            print("Position: ", Pos1)

        previous_error = current_error
        PID_arr.append(PID)
        if len(PID_arr) == 1000:
            break

    plt.plot(PID_arr)
    plt.show()
    robot.stop()

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)
    # Now close the connection to V-REP:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')













import sim
import numpy as np
import time, math

inf = 99999

class Lab:

    def __init__(self):
        sim.simxFinish(-1) 
        self.client_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
        if self.client_id != -1:
            print('Connected to V-REP')
        else:
            print('Connection to V-REP failed')
            exit()
        errorCode,self.left_motor_handle=sim.simxGetObjectHandle(self.client_id,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking)
        errorCode,self.right_motor_handle=sim.simxGetObjectHandle(self.client_id,'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking)
        res, self.frontLeftSonar = sim.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor3',sim.simx_opmode_blocking)
        res, self.frontRightSonar = sim.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor6',sim.simx_opmode_blocking)
        res, self.leftSonar = sim.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor1',sim.simx_opmode_blocking)
        res, self.leftSonar_b = sim.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor16',sim.simx_opmode_blocking)
        res, self.rightSonar = sim.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor8',sim.simx_opmode_blocking)
        res, self.rightSonar_b = sim.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor9',sim.simx_opmode_blocking)
        res, self.backLeftSonar = sim.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor15',sim.simx_opmode_blocking)
        res, self.backRightSonar = sim.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor10',sim.simx_opmode_blocking)
    
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(self.client_id,self.frontLeftSonar,sim.simx_opmode_streaming)
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(self.client_id,self.frontRightSonar,sim.simx_opmode_streaming)
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(self.client_id,self.leftSonar,sim.simx_opmode_streaming)
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(self.client_id,self.leftSonar_b,sim.simx_opmode_streaming)
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(self.client_id,self.rightSonar,sim.simx_opmode_streaming)
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(self.client_id,self.backLeftSonar,sim.simx_opmode_streaming)
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(self.client_id,self.backRightSonar,sim.simx_opmode_streaming)
        time.sleep(2)

    def findDist(self, objectHandle):
        res,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector = sim.simxReadProximitySensor(self.client_id,objectHandle,sim.simx_opmode_buffer)
        if detectionState == 1:
            return math.sqrt(sum(i**2 for i in detectedPoint))
        else:
            return inf

    def wallFollow(self):
        self.move(1.0, 1.0)
        if (self.findDist(self.frontLeftSonar) <= 1.0):
            self.move(2, -2)
        elif(self.findDist(self.frontRightSonar) <= 1.0):
            self.move(-2, 2)
        elif(self.findDist(self.leftSonar) <= 0.3):
            self.move(0.3, 0.1)
        elif(self.findDist(self.leftSonar) <= 1.0):
            self.move(0.1, 0.4)
        elif(self.findDist(self.leftSonar_b) <= 1.0):
            self.move(-5, 5)
        elif(self.findDist(self.rightSonar) <= 0.3):
            self.move(0.1, 0.3)
        elif(self.findDist(self.rightSonar) <= 1.0):
            self.move(0.4, 0.1)
        elif(self.findDist(self.rightSonar_b) <= 1.0):
            self.move(5, -5)
        elif(self.findDist(self.backLeftSonar) < 3.0):
            self.move(-15.0, 15.0)
        elif(self.findDist(self.backRightSonar) < 3.0):
            self.move(5.0, -5.0)
        else:
            vl = np.random.uniform(0.7, 0.9)
            vr = np.random.uniform(0.7, 0.9)
            self.move(vl, vr)

    def move(self, vl, vr):
        errorCode=sim.simxSetJointTargetVelocity(self.client_id,self.left_motor_handle , vl, sim.simx_opmode_blocking)
        errorCode=sim.simxSetJointTargetVelocity(self.client_id,self.right_motor_handle, vr, sim.simx_opmode_blocking)

def main():
    four = Lab()
    time.sleep(5)
    while True:
        four.wallFollow()

if __name__ == "__main__":
    main()

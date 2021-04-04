import numpy as np
import math as math
import time

class PB_Control():
    "This is an implementation of a high-level position controller"
    kPos = [[0.35,0.35,1.2,0.15,0.15,0.2]] #[[kpx, kpy, kpz, kdx, ...], [], ...]. This list includes the PD gains of all copters as well as the nominal gravity compensating thrust. Each copter gains are given in a sublist.
                       # Kp, Kd, Throttle_bias. Exp: 0.3 is mapped to around 1400 in RC commands. [[0.6,0.4,0.6,0.4,0.35,0.13], [0.6,0.4,0.6,0.4,0.5,0.2], [0.6,0.4,0.6,0.4,0.5,0.2]]

    scale = 1.1;
    for i in range(len(kPos)):
        for j in range(len(kPos[i])):
            kPos[i][j] = scale * kPos[i][j]
    print(kPos)
    Throttle_bias = 0.45 #(0 to 1) percentage throttle
    # kPos = [40*[0.35,0.35,0.9,0.15,0.15,0.2]] #[[kpx, kpy, kpz, kdx, ...], [], ...]. This list includes the PD gains of all copters as well as the nominal gravity compensating thrust. Each copter gains are given in a sublist.
    #                    # Kp, Kd, Throttle_bias. Exp: 0.3 is mapped to around 1400 in RC commands. [[0.6,0.4,0.6,0.4,0.35,0.13], [0.6,0.4,0.6,0.4,0.5,0.2], [0.6,0.4,0.6,0.4,0.5,0.2]]
    # Throttle_bias = 0.55 #(0 to 1) percentage throttle
    yawKp = 0.1  #1/sec
    maxYawRate = 1 #rad/sec
    maxRoll = math.pi/4
    maxPitch = math.pi/4
    maxThrottle = 0.99
    throttle = [] # in range (0,1). 1 being max throttle available and 0 being no throttle.
    roll = [] # rad
    pitch = [] # rad
    yawRate = [] # TBD
    fXYZ = []
    # Commands to be sent to MSP rxThread
    # MAX_THROTTLE_MSP = 2000
    # MIN_THROTTLE_MSP = 1000
    MAX_THROTTLE_CF = 60000
    MIN_THROTTLE_CF = 20000
    # RPY_OFFSET   = 1500
    throttleCF = []
    rollCF = []
    pitchCF = []
    yawRateCF = []
    mappedCommands = [] #List of lists. Each list contains roll, pitch, throttle and yaw rate for the respective drone
    desiredYaw = []
    initYawFlag = False
    for i in range (len(kPos)):
        throttle.append(0)
        roll.append(0)
        pitch.append(0)
        yawRate.append(0)
        throttleCF.append(1000)
        rollCF.append(1500)
        pitchCF.append(1500)
        yawRateCF.append(1500)
        mappedCommands.append([])
        desiredYaw.append(0)
        fXYZ.append([0, 0, 0])
        
    initTime = 0.0
    phase = 0
    
    def __init__(self):
        pass
    
    def ramp_up(self, lb, ub, duration, timer): # ramps up from lower bound (lb) to upper bound (ub) in a given time duration
        timer = timer - self.initTime
        throttleTemp = (ub - lb) * timer / duration + lb
        for i in range (len(self.throttle)):
            self.throttle[i] = throttleTemp
            self.roll[i] = 0.0
            self.pitch[i] = 0.0
    
    def ramp_down(self, lb, ub, duration, timer):
        timer = timer - self.initTime
        throttleTemp = (lb - ub) * timer / duration + ub
        for i in range (len(self.throttle)):
            self.throttle[i] = throttleTemp
            self.roll[i] = 0.0
            self.pitch[i] = 0.0
    
    def find_highLC(self,errors,gain): # The throttle bias is not included here.
        Kp_x = gain[0]; Kp_y = gain[1]; Kp_z = gain[2]; Kd_x = gain[3]; Kd_y = gain[4]; Kd_z = gain[5];
        ux = Kp_x * errors[0] + Kd_x * errors[3] 
        uy = Kp_y * errors[1] + Kd_y * errors[4]
        uz = Kp_z * errors[2] + Kd_z * errors[5]
        return [ux, uy, uz]

    def find_lowLC(self, highLC, yaw):
        epsilon = 0.000001
        delta = math.pi/720
        ux = highLC[0]
        uy = highLC[1]
        uz = highLC[2]
        # Finding throttle
        throttle = (ux**2 + uy**2 + uz**2)**(1/2)
        # Finding roll and pitch
        remainder = np.fmod(yaw, 2 * math.pi)
        if (abs(remainder - math.pi) <= delta) or ((abs(remainder + math.pi)) <= delta):
                roll = math.asin(uy/throttle)
                pitch = -math.atan(ux/uz)
        else:

                n_roll = (math.tan(yaw/2)**2 * throttle - (ux**2 * math.tan(yaw/2)**4 - 2 * ux**2 * math.tan(yaw/2)**2 + 4 * uy**2 * math.tan(yaw/2)**2 + 2 * uz**2 * math.tan(yaw/2)**2 + uz**2 * math.tan(yaw/2)**4 + ux**2 + uz**2 + 4 * ux * uy * math.tan(yaw/2) - 4 * ux * uy * math.tan(yaw/2)**3)**(1/2) + throttle)
                d_roll = (uy * math.tan(yaw/2)**2 - uy + 2 * ux * math.tan(yaw/2))
                if abs(d_roll) < epsilon:
                        roll = 0
                else:
                        roll = 2 * math.atan(n_roll/d_roll)
                # Finding pitch
                n_pitch = (uz - (ux**2 * math.tan(yaw/2)**4 - 2 * ux**2 * math.tan(yaw/2)**2 + 4 * uy**2 * math.tan(yaw/2)**2 + 2 * uz**2 * math.tan(yaw/2)**2 + uz**2 * math.tan(yaw/2)**4 + ux**2 + uz**2 + 4 * ux * uy * math.tan(yaw/2) - 4 * ux * uy * math.tan(yaw/2)**3)**(1/2) + uz * math.tan(yaw/2)**2)
                d_pitch = (ux - ux * math.tan(yaw/2)**2 + 2 * uy * math.tan(yaw/2))
                if abs(d_pitch) < epsilon:
                        pitch = 0
                else:
                        pitch = -2 * math.atan(n_pitch/d_pitch)
                # print("n_roll")
                # print(n_roll)
                # print("d_roll")
                # print(d_roll)
                # print("n_pitch")
                # print(n_pitch)
                # print("d_pitch")
                # print(d_pitch)
        return [throttle, roll, pitch]
        
    def saturate(self, variable, maxVal):
        if variable >  abs(maxVal) or -variable > abs(maxVal):
            variable = abs(maxVal) * np.sign(variable)
        return variable
    
    def map_commands(self):
        " This maps the commands to what crazyflie is expecting to receive"
        for i in range (len(self.throttleCF)):
            self.throttleCF[i] = int(self.throttle[i] * (self.MAX_THROTTLE_CF - self.MIN_THROTTLE_CF) + self.MIN_THROTTLE_CF) # This is mapped to an RC command in the range of 1100 to 2000.
            # self.rollCF[i] = int(self.roll[i] * (180 / math.pi)) # This is in decimal degrees and is half of the roll command since it gets multiplied by a factor of 2 in cleanflight
            # self.pitchCF[i] = int(self.pitch[i] * (180 / math.pi)) # 1000 is added becuase of type constraints. The numbers need to be unsigned.
            # self.yawRateCF[i] = int(-self.yawRate[i] * (180 / math.pi)) # This is half yawRate in decimal degrees per second. Stupid NOTE: MSP yaw rate interpretes the rate with a negative sign.
            self.rollCF[i] = (self.roll[i] * (180 / math.pi)) # This is in decimal degrees and is half of the roll command since it gets multiplied by a factor of 2 in cleanflight
            self.pitchCF[i] = (self.pitch[i] * (180 / math.pi)) # 1000 is added becuase of type constraints. The numbers need to be unsigned.
            self.yawRateCF[i] = (-self.yawRate[i] * (180 / math.pi)) # This is half yawRate in decimal degrees per second. Stupid NOTE: MSP yaw rate interpretes the rate with a negative sign.
##            print(self.yawRateCF[i])
            self.mappedCommands[i] = [self.rollCF[i], self.pitchCF[i], self.throttleCF[i], self.yawRateCF[i]]
    
##    def updateCompensationFactor(self, errorZ, timer): # The correct way of doing this is using the projection functions
##        if (self.adaptationInitFlag == False):
##            self.adaptationTime = timer
##            self.adaptationInitFlag = True
##        deltaT = timer - self.adaptationTime
##        self.compensationFactor += self.adaptationRate * errorZ * deltaT
##        self.compensationFactor = self.saturate(abs(self.compensationFactor - 1.0), 0.6) + 1.0
##        self.adaptationTime = timer

    def control_allocation(self, timer, yaw, errors, phase, rampUpDuration, rampDownDuration):
        if (self.initYawFlag == False):
            self.desiredYaw = yaw
            self.initYawFlag = True
        # Resetting the initTime if the phase has changed
        if self.phase != phase:
            self.initTime = timer
        self.phase = phase # Update the controller phase
        if phase == 0: # Remain on the floor please!
            for i in range (len(self.throttle)):
                self.throttle[i] = 0.0
                self.roll[i] = 0.0
                self.pitch[i] = 0.0
        
        elif phase == 1: #Ramp up
            self.ramp_up(0.0, self.Throttle_bias, rampUpDuration, timer)
        
        elif phase == 2: # Controllers in the loop case
            # Find high-level commands
##            self.updateCompensationFactor(errors[2], timer)
            for i in range (len(errors)): # errors = [[e_x, e_y, e_z, e_xd, e_yd, e_zd],[],...]
                highLC = self.find_highLC(errors[i], self.kPos[i])
                sat_ux = self.saturate(highLC[0], self.Throttle_bias)
                sat_uy = self.saturate(highLC[1], self.Throttle_bias)
                sat_uz = self.saturate(highLC[2], 2 * self.Throttle_bias) + self.Throttle_bias #: TO DO for adaptive gravity compensation
                sat_highLC = [sat_ux, sat_uy, sat_uz]
                self.fXYZ[i] = sat_highLC
                #print ('Saturated high-level commands are', sat_highLC)
                # Find Low-level commands
                lowLC = self.find_lowLC(sat_highLC, yaw[i])
                self.throttle[i] = self.saturate(lowLC[0], self.maxThrottle)
                self.roll[i] = self.saturate(lowLC[1], self.maxRoll) # This is in radians.
                self.pitch[i] = self.saturate(lowLC[2], self.maxPitch) # This is in radians.
                self.yawRate[i] = self.saturate(self.yawKp * (0 - yaw[i]), self.maxYawRate) #Radian per second
            
        elif phase == 3: # Ramp down
            self.ramp_down(0.0, self.Throttle_bias, rampDownDuration, timer)
            
        self.map_commands() #Sets the mappedCommands list.


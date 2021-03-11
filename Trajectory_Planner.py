import math
import numpy as np
from scipy import signal

class Trajectory_Planner:

        deadTime = 0   # During deadtime operation phase remains 0 which means no ramp up no closed-loop control
        rampUpDuration = 2 #During this period operation phase is 1 and props ramp up still no closed-loop control
        trajStartDelay = 2 #During this period operation phase is 2 (closed-loop control) but trajectories have not started yet. (Desired position is set to [0, 0, 0])
        rampDownDuration = 0 #During this period operation phase is 3 and props ramp down no closed-loop control.
        expTimeTotal = 0 #The totall time from expTime = 0 (absolute zero) to the end of trajectories (landing).
        trajType = 2 #set to 1 for set points and to 2 for smooth polynomials.
        phase = 0 

        setPointCoordinates = [[0,0,0,0], [0,0,1.5,5], [0,-0.9,1.5,5], [0,-0.9,1.5,5], [0,-0.9,0,5]] # The sequence of coordinates for the leader copter to get to at a certain time, [x,y,z, Time] all in the world frame!
                                                                         # NOTE: the initial coordinate of the leader is 0 0 0.
        coffsX = []
        coffsY = []
        coffsZ = []
        xOffsets = [0.0, 1.2] # X coordinates of all copters w.r.t the leader [0, 1.2, 0.5]
        yOffsets = [0.0, 0.0] # Y coordinates of all copters w.r.t the leader   [0, 0, -0.8]
        errors = []    # [[],[], ...]. This is a list of lists containing the position/velocity errors of each copter.
        desiredPose  = [0, 0, 0]  # Desired position of the leader copter
        desiredVel   = [0, 0, 0]  # Desired position of the leader copter
        desiredAccel = [0, 0, 0] #This list contains the desired acceleration of the copters. Since copter trajectories are the same except for just an offset, this is one list but not a list of lists.
        
        ARM_FLAG = True         #Commands to be sent to MSP rxThread
        FAILSAFE_FLAG = False
        maxPositionErrorTolerance = 0.5*3 #meter

        index = 1 #Indicates the current number of piece-wise trajectories past sofar
        expTimeElapsed = deadTime + rampUpDuration + trajStartDelay  #Experiment time elapsed at the begining of trajectory generation
        
        def __init__(self):
                timeFinal = self.deadTime + self.rampUpDuration + self.trajStartDelay # trajStartDelay is becuase of having the controller in the loop before the traj starts
                for i in range (len(self.setPointCoordinates)-1):
                        timeInitial = timeFinal
                        timeFinal +=  self.setPointCoordinates[i+1][3]
                        self.coffsX.append(self.traj_coeffs([ timeInitial, timeFinal, self.setPointCoordinates[i][0], self.setPointCoordinates[i+1][0] ]))
                        self.coffsY.append(self.traj_coeffs([ timeInitial, timeFinal, self.setPointCoordinates[i][1], self.setPointCoordinates[i+1][1] ]))
                        self.coffsZ.append(self.traj_coeffs([ timeInitial, timeFinal, self.setPointCoordinates[i][2], self.setPointCoordinates[i+1][2] ]))
                        
                self.expTimeTotal = timeFinal # The totall time from expTime = 0 (absolute zero) to the end of trajectories (landing).
                print("Total experiment time is ", self.expTimeTotal, "seconds")
                for i in range (len(self.xOffsets)):
                        self.errors.append([])

        
        def traj_coeffs(self, info):
                "Derives the trajectory coefficients for a 3rd order polynomial"
                Ti = info[0]  # Ti and Tf are absolute times w.r.t the begining of the test.
                Tf = info[1]
                qi = info[2]
                qf = info[3]
                A_Matrix = np.matrix([[1.0, Ti, Ti**2, Ti**3],[0.0, 1.0, 2.0*Ti, 3.0*Ti**2]
                             ,[1.0, Tf, Tf**2, Tf**3],[0.0, 1.0, 2.0*Tf, 3.0*Tf**2]])
                B_Vector = np.matrix([qi, 0.0, qf, 0.0])
                coffsTemp = np.linalg.inv(A_Matrix) * B_Vector.transpose()
                coffs = [coffsTemp.item(0), coffsTemp.item(1), coffsTemp.item(2), coffsTemp.item(3)]
                return  coffs # This will result in a list      
                
        def generate(self, expTime, position, velocity): # position = [[x, y, z],[], ...], velocity = [[xd, yd, zd],[], ...]
                "Generates desired trajectory, sets the phase, and returns desired positions, velocities and acceleration"
                ## Determining the operation phase
                if (expTime <= self.deadTime):
                        pass
                elif (expTime <= self.deadTime + self.rampUpDuration): # ramp up
                        self.phase = 1
                elif (expTime <=  self.expTimeTotal + self.trajStartDelay): # Feedback control is in the loop. # 2 seconds before/after the trajectories start/end.
                        self.phase = 2
                elif (expTime <=  self.expTimeTotal + self.trajStartDelay + self.rampDownDuration): # Ramp down
                        self.phase = 3
                else:
                        self.phase = 0
                        self.ARM_FLAG = False        
                ## Planning the trajectories
                if (self.trajType == 1): # Set Points
                        if (expTime < self.deadTime + self.rampUpDuration + self.trajStartDelay): #Stay at the very first commanded coordinates
                                DX = self.setPointCoordinates[0][0]
                                DY = self.setPointCoordinates[0][1]
                                DZ = self.setPointCoordinates[0][2]
                                DXd = 0; DYd = 0; DZd = 0;
                                DXdd = 0; DYdd = 0; DZdd = 0;
                        elif (expTime < self.expTimeTotal):
                                if ( self.setPointCoordinates[self.index][3] > expTime - self.expTimeElapsed):
                                        DX = self.setPointCoordinates[self.index][0]
                                        DY = self.setPointCoordinates[self.index][1]
                                        DZ = self.setPointCoordinates[self.index][2]
                                        DXd = 0; DYd = 0; DZd = 0;
                                        DXdd = 0; DYdd = 0; DZdd = 0;
                                else:
                                        #Have to define desired trajectories in the else case
                                        DX = self.setPointCoordinates[self.index][0]
                                        DY = self.setPointCoordinates[self.index][1]
                                        DZ = self.setPointCoordinates[self.index][2]
                                        DXd = 0; DYd = 0; DZd = 0;
                                        DXdd = 0; DYdd = 0; DZdd = 0;
                                        self.expTimeElapsed += self.setPointCoordinates[self.index][3]
                                        self.index += 1
                                        self.expTimeElapsed += self.setPointCoordinates[self.index][3]
                                        self.index += 1 #Note that this equates len(setPointCoordinates) at the end.
                        else:   #Stay at the very last commanded coordinates
                                DX = self.setPointCoordinates[-1][0]
                                DY = self.setPointCoordinates[-1][1]
                                DZ = self.setPointCoordinates[-1][2]
                                DXd = 0; DYd = 0; DZd = 0;
                                DXdd = 0; DYdd = 0; DZdd = 0;
                                
                else:  # Polynomials
                        if (expTime < self.deadTime + self.rampUpDuration + self.trajStartDelay): #Stay at the very first commanded coordinates
                                DX = self.setPointCoordinates[0][0]
                                DY = self.setPointCoordinates[0][1]
                                DZ = self.setPointCoordinates[0][2]
                                DXd = 0; DYd = 0; DZd = 0;
                                DXdd = 0; DYdd = 0; DZdd = 0;     
                        elif (expTime < self.expTimeTotal):
                                if ( self.setPointCoordinates[self.index][3] > expTime - self.expTimeElapsed):
                                        derivativeEnable = 0
                                        DX = self.coffsX[self.index-1][0] + self.coffsX[self.index-1][1] * expTime + self.coffsX[self.index-1][2] * expTime ** 2 + self.coffsX[self.index-1][3] * expTime ** 3
                                        DY = self.coffsY[self.index-1][0] + self.coffsY[self.index-1][1] * expTime + self.coffsY[self.index-1][2] * expTime ** 2 + self.coffsY[self.index-1][3] * expTime ** 3
                                        DZ = self.coffsZ[self.index-1][0] + self.coffsZ[self.index-1][1] * expTime + self.coffsZ[self.index-1][2] * expTime ** 2 + self.coffsZ[self.index-1][3] * expTime ** 3
                                        DXd = derivativeEnable*(self.coffsX[self.index-1][1] + 2 * self.coffsX[self.index-1][2] * expTime + 3 * self.coffsX[self.index-1][3] * expTime ** 2)
                                        DYd = derivativeEnable*(self.coffsY[self.index-1][1] + 2 * self.coffsY[self.index-1][2] * expTime + 3 * self.coffsY[self.index-1][3] * expTime ** 2)
                                        DZd = derivativeEnable*(self.coffsZ[self.index-1][1] + 2 * self.coffsZ[self.index-1][2] * expTime + 3 * self.coffsZ[self.index-1][3] * expTime ** 2)
                                        DXdd = derivativeEnable*(2 * self.coffsX[self.index-1][2] + 6 * self.coffsX[self.index-1][3] * expTime)
                                        DYdd = derivativeEnable*(2 * self.coffsY[self.index-1][2] + 6 * self.coffsY[self.index-1][3] * expTime)
                                        DZdd = derivativeEnable*(2 * self.coffsZ[self.index-1][2] + 6 * self.coffsZ[self.index-1][3] * expTime)
                                else:
                                        #Have to define desired trajectories in the else case
                                        DX = self.setPointCoordinates[self.index][0]
                                        DY = self.setPointCoordinates[self.index][1]
                                        DZ = self.setPointCoordinates[self.index][2]
                                        DXd = 0; DYd = 0; DZd = 0;
                                        DXdd = 0; DYdd = 0; DZdd = 0;
                                        self.expTimeElapsed += self.setPointCoordinates[self.index][3]
                                        self.index += 1
                        else:   #Stay at the very last commanded coordinates
                                DX = self.setPointCoordinates[-1][0]
                                DY = self.setPointCoordinates[-1][1]
                                DZ = self.setPointCoordinates[-1][2]
                                DXd = 0; DYd = 0; DZd = 0;
                                DXdd = 0; DYdd = 0; DZdd = 0;   
                
                # Position/velocity errors [e_x, e_y, e_z, e_xd, e_yd, e_zd]
                for i in range (len(self.xOffsets)):
                        self.errors[i] = ([DX + self.xOffsets[i] - position[i][0], DY + self.yOffsets[i] - position[i][1], DZ - position[i][2],
                                           DXd - velocity[i][0], DYd - velocity[i][1], DZd - velocity[i][2]]) #Leader first
                        #FailSafe Check
                        for j in range (0,3):
                                if (abs(self.errors[i][j]) > self.maxPositionErrorTolerance):
                                        self.FAILSAFE_FLAG = True
                                        # print("abs(self.errors[i][j]) > self.maxPositionErrorTolerance")
                                        # print("where i and j are", i, j)
                                        # print("and self.errors[i][j] is", self.errors[i][j])
                                # else:
                                #         print("abs(self.errors[i][j]) <= self.maxPositionErrorTolerance")
                                #         print("where i and j are", i, j)
                                #         print("and self.errors[i][j] is", self.errors[i][j])

                # Desired position, velocity, and acceleration of the head copter
                self.desiredPose  = [DX, DY, DZ]
                self.desiredVel   = [DXd, DYd, DZd]
                self.desiredAccel = [DXdd, DYdd, DZdd]

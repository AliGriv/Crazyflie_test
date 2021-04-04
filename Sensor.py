import math
import numpy as np
from scipy import signal
from Velocity_Filter import Velocity_Filter

class Sensor:
    ##FailSafe Related##
    trackLostCounter = []
    FailSafeLostCounts = 10
    FAILSAFE_FLAG = False # INTERFACE ATTRIBUTE

    #FeedBack Related##
    initFlag = False
    initLeaderPosition = [0, 0, 0]
    Position = [] # INTERFACE ATTRIBUTE. A list of lists; each of sublist contains the position of a copter in the world frame. Initializes based on camera measurements.
    Velocity = [] # INTERFACE ATTRIBUTE. Contains all velocity vectors of all copters. List of lists. Initializes at zeros.
    tempVel = []

    ##Yaw Related##
    initYaw = []
    yaw = []
    yawCounter = []
    yawFiltered = [] #INTERFACE ATTRIBUTE. Initializes based on camera measuremnts.
    
    ##Filter related##
    firFilter0 = Velocity_Filter(order = 5, cutoff = 0.05, channelNum = 3) #"Numerical derivation and filtering of the camera measurements"
    firFilter1 = Velocity_Filter(order = 5, cutoff = 0.05, channelNum = 3) #"Numerical derivation and filtering of the camera measurements"
    firFilter2 = Velocity_Filter(order = 5, cutoff = 0.05, channelNum = 3) #"Numerical derivation and filtering of the camera measurements"
    nonlinFilterThreshold =  0.1 # m/s

    def __init__(self, numCopters):
        for i in range (numCopters):
            self.Position.append([]) # Positions will be initialized with the actual measurements later.
            self.Velocity.append([0,0,0]) # Velocities initialized at 0s.
            self.tempVel.append([0,0,0])
            self.yaw.append(0)
            self.yawCounter.append(0) # Yaw conters initialized at 0s.
            self.yawFiltered.append(0) # Yaw will be initialized with the actual measurements later.
            self.trackLostCounter.append(0)
            
    def failSafe(self, trackingFlag): # This is a list of tracking indexes
        for i in range (len(trackingFlag)):
            if (trackingFlag[i] == False):
                self.trackLostCounter[i] += 1
                if(self.trackLostCounter[i] > self.FailSafeLostCounts):
                    self.FAILSAFE_FLAG = True
            else:
                self.trackLostCounter[i] = 0

    def find_yaw(self, orientations, trackingFlag): # The last element of orientation is the scalar one. List of lists
            "Extraction of the copter's yaw wrt the world frame from the camera measured quaternions"
            for i in range (len(orientations)):
                if (trackingFlag[i] == True):
                    q0 = orientations[i][3]
                    q1 = orientations[i][0]
                    q2 = orientations[i][1]
                    q3 = orientations[i][2]
                    r11 = 1 - 2 * (q2**2 + q3**2)
                    r31 = 2 * (q1 * q3 - q0 * q2)   
                    yaw = math.atan2(-r31,r11)
                    # initializing yawFiltered
                    if (self.initFlag == False):
                            self.yawFiltered[i] = yaw
                    
                    # Compensating for yaw overflows
                    if (yaw - self.yaw[i] < -5.25): # approx -300 degrees
                            self.yawCounter[i] += 1
                    elif (yaw - self.yaw[i] > 5.25):
                            self.yawCounter[i] += -1
           
                    self.yaw[i] = yaw
                    yaw = yaw + 2 * math.pi * self.yawCounter[i]

                    #Filtering possible spikes
                    if(abs(yaw - self.yawFiltered[i]) < .05):
                            self.yawFiltered[i] = yaw
                        
    def setPosition(self, Position, trackingFlag): #Sets all positions by accounting for the initial position offset. Leader is placed in the origin of the world frame!
        for i in range (len(Position)):
            if (trackingFlag[i] == True):
                self.Position[i] = [(Position[i][j] - self.initLeaderPosition[j]) for j in range (0,3)]
        
    def estimateVel(self):
        for i in range (len(self.Position)):
            tempVel = eval('self.firFilter' + str(i) + '.numDiff_FIR(self.Position[i])')
            for j in range (len(tempVel)):  #Nonlinear filtering (spike remover)
                            if (abs(self.tempVel[i][j] - tempVel[j]) < self.nonlinFilterThreshold):
                                self.Velocity[i][j] = tempVel[j]
                            self.tempVel[i][j] = tempVel[j]
                        
    def process(self, Position, Orientation, trackingFlag): # Position and Orientation are lists of lists and trackingFlag is a list.
        #Initializing measurements
        if (self.initFlag == False):
            allTracked = True
            for i in range (len(trackingFlag)):
                allTracked = allTracked * trackingFlag[i]
            if (allTracked == True):
                self.initLeaderPosition = Position[0]    #Reading the leader initial position
                self.setPosition(Position, trackingFlag) #Initializing positions
                self.find_yaw(Orientation, trackingFlag) #Initializing yaws
                self.initFlag = True
                print("Copter position/orientation coordinates initialized")
                
        else:
            self.find_yaw(Orientation, trackingFlag) #self.yawFiltered gets updated only for the copters that have been tracked
            self.setPosition(Position, trackingFlag) #self.Position gets updated only for the copters that have been tracked
            self.estimateVel()                       #self.Velocity gets updated
            self.failSafe(trackingFlag) #If any of the copters is lost for more than 10 consecutive samples, the system goes into a failsafe mode.

import math
import numpy as np
from scipy import signal

class Path_Generator:
        
        initTime = 0.0 # The time when position is initialized
        initPosition = np.matrix([0.0, 0.0, 0.0]) #wrt camera frame

        position = np.matrix([0.0, 0.0, 0.0]) #Position wrt the camera frame
        positionOld = np.matrix([0.0, 0.0, 0.0])
        positionOldOld = np.matrix([0.0, 0.0, 0.0])
        positionFilteredOld = np.matrix([0.0, 0.0, 0.0])
        velocity = np.matrix([0.0, 0.0, 0.0]) #Velocity wrt the camera frame
        velocityOld = np.matrix([0.0, 0.0, 0.0]) # This is to implement a median filter on velocity estimates
        velocityOldOld = np.matrix([0.0, 0.0, 0.0])
        cameraTime = 0.0 #Camera time

        yaw = 0.0
        yawCounter = 0
        yawFiltered = 0.0
        yawRate = int(0.0 + 1500)
        
        phase = 0 
        
        ZC1 = [0.0, 0.0, 0.0, 0.0]
        ZC2 = [0.0, 0.0, 0.0, 0.0]
        
        rampUpDuration = 4.0 #seconds
        rampDownDuration = 1.0 #seconds
        trajStartTime = 0.0
        trajDuration = 0.0
        hoverDuration = 0.0
        zFinal = 0.0
        initFlag = False
        
        ARM_FLAG = True         #Commands to be sent to MSP rxThread
        FAILSAFE_FLAG = False
        trackLostCounter = 0
        
        def __init__(self, trajStartTime, trajDuration, hoverDuration, zFinal):
        
                self.trajStartTime = trajStartTime
                self.trajDuration = trajDuration
                self.hoverDuration = hoverDuration
                self.zFinal = zFinal
        
        def traj_coeffs(self, input):
                "Derives the trajectory coefficients for a 3rd order polynomial"
                Ti = input[0]
                Tf = input[1]
                qi = input[2]
                qf = input[3]
                A_Matrix = np.matrix([[1.0, Ti, Ti**2, Ti**3],[0.0, 1.0, 2.0*Ti, 3.0*Ti**2]
                             ,[1.0, Tf, Tf**2, Tf**3],[0.0, 1.0, 2.0*Tf, 3.0*Tf**2]])
                B_Vector = np.matrix([qi, 0.0, qf, 0.0])
                return np.linalg.inv(A_Matrix) * B_Vector.transpose() # This will result in a column vector
                
        def filtering(self, position, cameraTime, TRACK_FLAG):
                "Numerical derivation and hard filtering of the Camera measurements"
                if (TRACK_FLAG): # Estimate the velocity only if the copter is tracked by the cameras.
                        pos = np.matrix([position[0], position[1], position[2]])
                        #self.position = pos
                        ## Applying a median filter on position
                        tempX = [self.positionOldOld.item(0), self.positionOld.item(0), pos.item(0)]
                        tempZ = [self.positionOldOld.item(1), self.positionOld.item(1), pos.item(1)]
                        tempY = [self.positionOldOld.item(2), self.positionOld.item(2), pos.item(2)]
                        # Updating the old values
                        self.positionOldOld = self.positionOld
                        self.positionOld = pos
                        # Filter and update the position
                        self.position = np.matrix([signal.medfilt(tempX,3)[2], signal.medfilt(tempZ,3)[2], signal.medfilt(tempY,3)[2]])
                                
                        ## Differentiation
                        cameraDt = cameraTime - self.cameraTime
                        if cameraDt < 0.008:
                                #print(cameraDt)
                                cameraDt = 0.008
                        vel = (self.position - self.positionFilteredOld)/cameraDt
                        # Update time and position and reset counter
                        self.positionFilteredOld = self.position ##Note that self.position and self.positionOld are both filtered data
                        self.cameraTime = cameraTime
                        self.trackLostCounter = 0

                        ## Applying a median filter on velocity
                        tempX = [self.velocityOldOld.item(0), self.velocityOld.item(0), vel.item(0)]
                        tempZ = [self.velocityOldOld.item(1), self.velocityOld.item(1), vel.item(1)]
                        tempY = [self.velocityOldOld.item(2), self.velocityOld.item(2), vel.item(2)]
                        # Updating the old values
                        self.velocityOldOld = self.velocityOld
                        self.velocityOld = vel
                        # Filter and update the velocity
                        self.velocity = np.matrix([signal.medfilt(tempX,3)[2], signal.medfilt(tempZ,3)[2], signal.medfilt(tempY,3)[2]])

                                
                else:
                        self.trackLostCounter += 1
                        
        def find_yaw(self, orientation): # The last element of orientation is the scalar one
                "Extraction of the copter's yaw wrt the world frame from the camera measured quaternions"
                q0 = orientation[3]
                q1 = orientation[0]
                q2 = orientation[1]
                q3 = orientation[2]
                r11 = 1 - 2 * (q2**2 + q3**2)
                r31 = 2 * (q1 * q3 - q0 * q2)   
                yaw = math.atan2(-r31,r11)
                # initializing yawFiltered
                if (self.initFlag == False):
                        self.yawFiltered = yaw
                
                # Compensating for yaw overflows
                if (yaw - self.yaw < -5.25): # approx -300 degrees
                        self.yawCounter += 1
                elif (yaw - self.yaw > 5.25):
                        self.yawCounter += -1
       
                self.yaw = yaw
                yaw = yaw + 2 * math.pi * self.yawCounter

                #Filtering possible spikes
                if(abs(yaw - self.yawFiltered) < .05):
                        self.yawFiltered = yaw
        
        def transform(self, vector):
                "Transforms a vector defined in the camera frame to the world frame"
                cameraToworld = np.matrix([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
                transformedVector = cameraToworld * vector.transpose() # This yields a coloumn vector
                return transformedVector.transpose()
                
        def generate(self, position, orientation, cameraTime, TRACK_FLAG, time):
                "Generates desired trajectory, sets the phase, yawRate and returns position/velocity errors as well as yaw"
                expTime = time - self.initTime # Time since the copter was tracked first and position was initialized.
                
                # Estimate velocity
                self.filtering(position, cameraTime, TRACK_FLAG)
                
                # Applying the frame adjustments
                initX = self.initPosition.item(0) #Coordinate frame adjustment is considered(Camera to world)
                initY = -self.initPosition.item(2)
                initZ = self.initPosition.item(1)
                posFeedback = self.transform(self.position)
                velFeedback = self.transform(self.velocity)
                
                ## Desired trajectory generation
                if not self.initFlag:
                        self.initZcUp = self.traj_coeffs( [self.trajStartTime, self.trajStartTime + 3.0,
                                                                initZ, initZ + 0.3] )
##                        self.initZcDown = self.traj_coeffs( [self.trajStartTime + 3.0 + 4.0 + 2.0 * self.trajDuration + self.hoverDuration,
##                                                             self.trajStartTime + 6.0 + 4.0 + 2.0 * self.trajDuration + self.hoverDuration,
##                                                                initZ + 0.3, initZ] )
                        self.initZcDown = self.traj_coeffs( [self.trajStartTime + 3.0 + 4.0 + 16,
                                                             self.trajStartTime + 6.0 + 4.0 + 16 ,
                                                                initZ + 0.3, initZ] )
                        self.ZC1 = self.traj_coeffs( [self.trajStartTime + 3 + 2, self.trajStartTime + 3.0 + 2.0 + self.trajDuration,
                                                                initZ + 0.3, initZ + 0.3 + self.zFinal] )
                        self.ZC2 = self.traj_coeffs( [self.trajStartTime + 3 + 2 + self.trajDuration + self.hoverDuration,
                                                      self.trajStartTime + 3 + 2 + 2 * self.trajDuration + self.hoverDuration,
                                                                initZ + 0.3 + self.zFinal, initZ + 0.3] )
                        self.XC1 = self.traj_coeffs([self.trajStartTime + 3.0 + self.trajDuration + 2.0, self.trajStartTime + self.trajDuration + 5 + 3.0,
                                                     initX, initX + 2.0]) ## This is for the moving on a line maneuver
                        self.thetaUp = self.traj_coeffs([self.trajStartTime + 3 + 2, self.trajStartTime + 3 + 2 + 3 * 8, 0.0, 3 * 2 * math.pi])
                        self.thetaDown = self.traj_coeffs([self.trajStartTime + 3 + 2 + 3 * 8, self.trajStartTime + 3 + 2 + 6 * 8, 0.0, -3 * 2 * math.pi])
                        self.thetaVertical = self.traj_coeffs([self.trajStartTime + 3 + 2 , self.trajStartTime + 3 + 2 + 2 * 8, 0.0, 2 * 2 * math.pi])
                        self.initFlag = True
                #DPsi = 0.0
                        
                ## Desired X generation
                R = 0.75
                if (expTime <= self.trajStartTime + 5):
                      Dx = initX
                      Dxd = 0.0
                elif (expTime <= self.trajStartTime + 5 + 16):
                        Dtheta = self.thetaVertical.item(0) + self.thetaVertical.item(1) * expTime + self.thetaVertical.item(2) * expTime ** 2 + self.thetaVertical.item(3) * expTime ** 3
                        Dthetad = self.thetaVertical.item(1) + 2 * self.thetaVertical.item(2) * expTime + 3 * self.thetaVertical.item(3) * expTime ** 2
                        Dx = initX - R * math.sin(Dtheta)
                        Dxd = -R * math.cos(Dtheta) * Dthetad
                else:
                        Dx = initX
                        Dxd = 0.0
##                R = 0.75
##                if (expTime <= self.trajStartTime + 3 + 2):
##                        Dx = initX
##                        Dxd = 0.0
##                        Dy = initY
##                        Dyd = 0.0
##                elif (expTime <= self.trajStartTime + 3 + 2 + 3 * 8):
##                        Dtheta = self.thetaUp.item(0) + self.thetaUp.item(1) * expTime + self.thetaUp.item(2) * expTime ** 2 + self.thetaUp.item(3) * expTime ** 3
##                        Dthetad = self.thetaUp.item(1) + 2 * self.thetaUp.item(2) * expTime + 3 * self.thetaUp.item(3) * expTime ** 2
##                        Dx = initX + R * (1 - math.cos(Dtheta))
##                        Dxd = R * math.sin(Dtheta) * Dthetad
##                        Dy = initY - R * math.sin(Dtheta)
##                        Dyd = -R * math.cos(Dtheta) * Dthetad
##                elif (expTime <= self.trajStartTime + 3 + 2 + 6 * 8):
##                        Dtheta = self.thetaDown.item(0) + self.thetaDown.item(1) * expTime + self.thetaDown.item(2) * expTime ** 2 + self.thetaDown.item(3) * expTime ** 3
##                        Dthetad = self.thetaDown.item(1) + 2 * self.thetaDown.item(2) * expTime + 3 * self.thetaDown.item(3) * expTime ** 2
##                        Dx = initX + R * (1 - math.cos(Dtheta))
##                        Dxd = R * math.sin(Dtheta) * Dthetad
##                        Dy = initY - R * math.sin(Dtheta)
##                        Dyd = -R * math.cos(Dtheta) * Dthetad
##                else:
##                        Dx = initX
##                        Dxd = 0.0
##                        Dy = initY
##                        Dyd = 0.0
                
                ## Desired Y generation
                Dy = initY
                Dyd = 0.0
                
                ## Desired altitude generation
                # Remain on the floor
                if (expTime <= self.trajStartTime):
                        Dz = initZ
                        Dzd = 0.0
                # Escape from the ground effect
                elif (expTime <= self.trajStartTime + 3.0):
                        Dz = self.initZcUp.item(0) + self.initZcUp.item(1) * expTime + self.initZcUp.item(2) * expTime ** 2 + self.initZcUp.item(3) * expTime ** 3
                        Dzd = self.initZcUp.item(1) + 2 * self.initZcUp.item(2) * expTime + 3 * self.initZcUp.item(3) * expTime ** 2
                # Hover for 2 seconds
                elif (expTime <= self.trajStartTime + 3.0 + 2.0):
                        Dz = initZ + 0.3
                        Dzd = 0.0
                elif (expTime <= self.trajStartTime + 3.0 + 2.0 + 16):        
                        Dtheta = self.thetaVertical.item(0) + self.thetaVertical.item(1) * expTime + self.thetaVertical.item(2) * expTime ** 2 + self.thetaVertical.item(3) * expTime ** 3
                        Dthetad = self.thetaVertical.item(1) + 2 * self.thetaVertical.item(2) * expTime + 3 * self.thetaVertical.item(3) * expTime ** 2
                        Dz = initZ +.3 + R * (1 - math.cos(Dtheta))
                        Dzd = R * math.sin(Dtheta) * Dthetad
##                # Elevate
##                elif (expTime <= self.trajStartTime + 3.0 + 2.0 + self.trajDuration):
##                        Dz = self.ZC1.item(0) + self.ZC1.item(1) * expTime + self.ZC1.item(2) * expTime ** 2 + self.ZC1.item(3) * expTime ** 3
##                        Dzd = self.ZC1.item(1) + 2 * self.ZC1.item(2) * expTime + 3 * self.ZC1.item(3) * expTime ** 2
##                # Hover
##                elif (expTime <= self.trajStartTime + 3.0 + 2.0 + self.trajDuration + self.hoverDuration):
##                        Dz = self.zFinal + initZ + 0.3
##                        Dzd = 0.0
##                # Land till 30 cm height
##                elif (expTime <= self.trajStartTime + 3.0 + 2.0 + 2.0 * self.trajDuration + self.hoverDuration):
##                        Dz = self.ZC2.item(0) + self.ZC2.item(1) * expTime + self.ZC2.item(2) * expTime ** 2 + self.ZC2.item(3) * expTime ** 3
##                        Dzd = self.ZC2.item(1) + 2 * self.ZC2.item(2) * expTime + 3 * self.ZC2.item(3) * expTime ** 2
                # Hover for 2 seconds
                elif (expTime <= self.trajStartTime + 3.0 + 4.0 + 16):
                        Dz = initZ + 0.3
                        Dzd = 0.0
                # Land
                elif (expTime <= self.trajStartTime + 6.0 + 4.0 + 16):
                        Dz = self.initZcDown.item(0) + self.initZcDown.item(1) * expTime + self.initZcDown.item(2) * expTime ** 2 + self.initZcDown.item(3) * expTime ** 3
                        Dzd = self.initZcDown.item(1) + 2 * self.initZcDown.item(2) * expTime + 3 * self.initZcDown.item(3) * expTime ** 2
                # Remain on the floor
                else :  
                        Dz = initZ
                        Dzd = 0.0
                        
                # Finding errors
                posDesired = np.matrix([Dx, Dy, Dz])
                velDesired = np.matrix([Dxd, Dyd, Dzd])
                errPosition = posDesired - posFeedback
                errVelocity = velDesired - velFeedback

                ## Failsafe check
                if (max(abs(errPosition.item(0)), abs(errPosition.item(1)), abs(errPosition.item(2)) ) > 0.5 or self.trackLostCounter > 100):
                        self.phase = 0
                        self.ARM_FLAG = False
                ## Determining the operation phase
                else:
                        if (expTime <= 1.0):
                                pass
                        elif (expTime <= 1.0 + self.rampUpDuration): # ramp up
                                self.phase = 1
                        elif (expTime <= self.trajStartTime + 6.0 + 2.0 * self.trajDuration + self.hoverDuration + 4.0): # Feedback control is in the loop. This happens after 5 secs of the start time. This ensures the controller is in the loop 2 seconds before the trajectory starts and gets out of the loop 2 secs after the trajectory ends.
                                self.phase = 2
                        elif (expTime <= self.trajStartTime + 6.0 + 2.0 * self.trajDuration + self.hoverDuration + 4.0 + self.rampDownDuration): # Ramp down
                                self.phase = 3
                        else:
                                self.phase = 0
                                self.ARM_FLAG = False
                     
                ## Find and update yaw
                self.find_yaw(orientation) # This is continuous, filtered and in radians.
                
                output = [errPosition.item(0), errPosition.item(1), errPosition.item(2), errVelocity.item(0), errVelocity.item(1), errVelocity.item(2), self.yawFiltered, posDesired.item(0), posDesired.item(1), posDesired.item(2)]
                return output

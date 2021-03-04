from scipy import signal
import numpy as np

class Velocity_Filter():

    #Channels - # of indpendant channels to be filtered (for pos x,y,z this is 3)
    def __init__(self, order=3, cutoff=0.1, channelNum=3):
        self.channelSize = channelNum
        self.FIR_coeffs = np.asmatrix(signal.firwin(order, cutoff))                  ##Get coefficients for FIR lowpass and put in vector form
        self.velDataRaw_m = np.asmatrix(np.zeros((channelNum, order)))               #Holds raw data (has a memory equal to order of filter)

        self.posOld_m = np.asmatrix(np.zeros((channelNum, 1)))
        self.velCur_m = np.asmatrix(np.zeros((channelNum, 1)))

        self.clock_prev = 0

    def numDiff_FIR(self, curChannelData):
        for i in range(0,len(curChannelData)):
            self.velCur_m[i,0] = (curChannelData[i] - self.posOld_m[i,0])/0.008     #Numerical differntiation to get current velocity
            self.posOld_m[i,0] = curChannelData[i]                                  #Update old position
        
        self.velDataRaw_m[:,1:self.velDataRaw_m.shape[1]] = self.velDataRaw_m[:, 0:self.velDataRaw_m.shape[1]-1]   #Shift all elements to the right by 1
        self.velDataRaw_m[:,0] = self.velCur_m                                                                     #New data placed in column 0
##        print((self.velCur_m.transpose())[0,:])
        filtVelocity = self.velDataRaw_m * self.FIR_coeffs.transpose()
##        print([filtVelocity.item(i) for i in range (0, self.channelSize)])
        return  [filtVelocity.item(i) for i in range (0, self.channelSize)] #Return the filtered velocity 

    def runFIR(self):
        return (self.velDataRaw_m * self.FIR_coeffs.transpose()).tolist()

    def runMED(self):
        temp = self.velDataRaw_m
        temp = np.sort(temp, axis=1)
        return temp[:,int(temp.shape[1]/2)].tolist() 

    

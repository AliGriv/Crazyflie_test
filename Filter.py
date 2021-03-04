from scipy import signal
import numpy as np

class Filter():

    #Channels - # of indpendant channels to be filtered (for pos x,y,z this is 3)
    def __init__(self, order=3, cutoff=0.1, channelNum=3):
        self.channelSize   = channelNum
        self.FIR_coeffs    = np.array(signal.firwin(order, cutoff)).transpose()   # Get coefficients for FIR lowpass and put in vector form
        self.signalDataRaw = np.zeros((channelNum, order))  # Holds raw data (has a memory equal to order of filter)

    ## Run
    def runFIR(self, newData):
        self.signalDataRaw[:, 1:self.signalDataRaw.shape[1]] = self.signalDataRaw[:, 0:self.signalDataRaw.shape[1]-1] # Shift all elements to the right by 1
        # print(newData)
        # print(newData.shape)
        self.signalDataRaw[:, 0]                             = newData                                                # New data placed in column 0

        return np.dot(self.signalDataRaw, self.FIR_coeffs)

    def runMED(self):
        temp = self.velDataRaw_m
        temp = np.sort(temp, axis=1)
        return temp[:,int(temp.shape[1]/2)].tolist() 

    

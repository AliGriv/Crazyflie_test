from threading import Thread
import serial
import time
import struct
import random

class Cleanflight_MSP:

        #Send Packet Params
        dataToSend = ""         #Packed string of data to send
        dataGood = -1
        crc = 0                         #Checksum for packet

        #Receive Packet Params
        received_Length_raw     = ""            #<dataLength>,<msgCode>,<data>,<crc>
        received_msgCode_raw    = "" 
        received_data_raw       = ""
        received_CRC_raw        = ""    #0 if new data received, -1 otherwise
        
        received_dataGood               = -1

        received_Length_unpacked        = 0             #<dataLength>,<msgCode>,<data>,<crc>
        received_msgCode_unpacked       = 0 
        received_data_unpacked          = 0
        received_CRC_unpacked           = 0     #0 if new data received, -1 otherwise

        ser_Flight = 0

        #Constant definitions for MSP_RX
        ARM = 1600
        DISARM = 1050
        SAFE = 1050
        FAILSAFE = 1600

        def __init__(self, port = 'COM3', baudrate = 115200):

                self.ser_Flight = serial.Serial(             #Set up connection from Flight controller to linux machine
                    port=port,
                    baudrate=baudrate,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS
                )

                self.ser_Flight.isOpen()

        #MULTIWII SERIAL PROTOCOL (MSP)
        #<preamble>,<direction>,<dataLength>,<msgCode>,<data>,<crc>
        # '$M' is the preamble to start a message (automatically generated)
        # direction     - '<' to the flight controller, '>' from the flight controller
        # dataLength    - Length of data in bytes (0-255)
        # msgCode       - message id defined in msp_protocol.h (0-255)
        # data                  - data is sent in little endian
        # crc                   - XOR of <dataLength>,<msgCode> and each data byte in a zero'ed sum 
        def sendMSP(self, direction, dataLength, msgCode, data, dataFormat):            #Send an MSP Command
                self.dataGood = self.checkData(dataLength, msgCode, data, dataFormat)
                #print self.dataGood

                self.calcCRC(self.dataToSend)

                if self.dataGood == 0:
                        packet = struct.pack('<3c', '$'.encode('utf-8'), 'M'.encode('utf-8'), direction.encode('utf-8')) + self.dataToSend + struct.pack('<B', self.crc)
                        packet_sent = self.ser_Flight.write(packet)

        def receiveMSP(self, dataFormat):
                initTime = time.time()  #Wait for response from FC (if no response in 0.1s, break the loop)
                self.received_dataGood = -1
                
                while time.time()-initTime < 0.1 and self.received_dataGood != 0:

                        header = self.ser_Flight.read()
                        if header == '$'.encode('utf-8'):
                                self.ser_Flight.read(2) #Get rid of the next 2 bytes as well
                                self.received_dataGood = 0

                #Read data
                self.received_Length_raw                = self.ser_Flight.read()
                self.received_Length_unpacked   = struct.unpack('<B', self.received_Length_raw)[0]      
                self.received_msgCode_raw               = self.ser_Flight.read()

                self.received_Length_unpacked = 8 ##FIX THIS HARDCODE
                
                self.received_data_raw                  = self.ser_Flight.read(self.received_Length_unpacked)
                self.received_CRC_raw                   = self.ser_Flight.read()

                
                
                #Unpack Data
                self.received_msgCode_unpacked  = struct.unpack('<B', self.received_msgCode_raw)[0]
                self.received_data_unpacked             = struct.unpack(dataFormat, self.received_data_raw)
                self.received_CRC_unpacked              = struct.unpack('<B',self.received_CRC_raw)[0]

                if self.received_CRC_unpacked == self.calcCRC(self.received_Length_raw + self.received_msgCode_raw + self.received_data_raw):
                        self.received_dataGood = 1
                else:
                        self.received_dataGood = -1

                return self.received_dataGood, self.received_data_unpacked

        #Ensure data parameters are consistent with each other
        #Return 0 if data good
        #-1 if datalength is wrong
        #-2 if data cannot be packed into struct (incorrect data formatting)
        def checkData(self, dataLength, msgCode, data, dataFormat):             
                #try:   
                self.dataToSend = struct.pack('<2B', dataLength, msgCode) + struct.pack(dataFormat, *data)

                if len(self.dataToSend) == dataLength + 2:                              
                        return 0                                                                                
                else:
                        return -1
                #except:
                        #return -2
                
        #Calculate data checksum
        def calcCRC(self, dataToCheck):
                self.crc = 0
                for i in dataToCheck:
                        self.crc= self.crc ^ ord(chr(i))

##        def rxThread_init(self):
##                rxThread = Thread(target = self.rxThread_run)
##                rxThread.start()
##
##        def rxThread_run(self):               
##                for i in range(1, 100):                                                #Start sending MSP RX with all stick in middle position (1550) and throttle stick down (1050) & DISARM #We must send a disarm command for about a second before arming
##                        commandsToGo = [1500, 1500, 1050, 1500, 1000, 1000, 1000, 1000]
##                        self.sendMSP('<', 16, 200, commandsToGo, '<8h')
##                        time.sleep(0.01)
##                for i in range(1, 20):                                                  #Start sending MSP RX with all stick in middle position (1550) and throttle stick down (1050) & ARM
##                        commandsToGo = [1500, 1500, 1050, 1500, 1600, 1000, 1000, 1000]
##                        self.sendMSP('<', 16, 200, commandsToGo, '<8h')
##                        time.sleep(0.01)
##
##                rxThread_rxInitialized_f = True
##
##                ## MAIN LOOP ##
##                while True:
##                        if(pathGenerator.ARM_FLAG == True and pathGenerator.FAILSAFE_FLAG == False):          #Sends command
##                                commandsToGo = [controller.rollMSP, controller.pitchMSP, controller.throttleMSP, pathGenerator.yawRateMSP, self.ARM, self.SAFE, 1000, 1000]
##                                self.sendMSP('<', 16, 200, commandsToGo, '<8h')
##                                time.sleep(0.01)
##                        elif(pathGenerator.ARM_FLAG == False and pathGenerator.FAILSAFE_FLAG == False):       #Controller Disarms FC
##                                commandsToGo = [1500, 1500, 1050, 1500, self.DISARM, self.SAFE, 1000, 1000]
##                                self.sendMSP('<', 16, 200, commandsToGo, '<8h')
##                                break
##                        else:   # Controller has issued FAILSAFE_FLAG = True
##                                commandsToGo = [1500, 1500, 1050, 1500, self.DISARM, self.FAILSAFE, 1000, 1000]
##                                self.sendMSP('<', 16, 200, commandsToGo, '<8h')
##                                break
##                        
##
#OBSELTE: note that currently mainloop does not allow rearming of FC since the rxThread_init is only called once.
                        
##MSP = Cleanflight_MSP('COM5', 115200) #Initialize MSP object

##while 1:
##      MSP.sendMSP('<', 0, 133, [], '<')  ##Get attitude
##      dataGood, data = MSP.receiveMSP('<14h')
##        print data[6:9]
##        time.sleep(0.02)
##        
##        for i in range(1050, 1200):
##                print(i)
##                MSP.sendMSP('<', 16, 214, [i, 1000, 1000, 1000, 1000, 1000, 1000, 1000], '8h')
##                time.sleep(0.04)


        #Fix exceptions

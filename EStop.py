import serial
import time
import struct
class EStop:
		
		armingState = 0;
		noDataCount = 0;

		def __init__(self, port = 'COM3', baudrate = 115200):

			self.ser_EStop = serial.Serial(             #Set up connection from Flight controller to linux machine
				port=port,
				baudrate=baudrate,
				parity=serial.PARITY_NONE,
				stopbits=serial.STOPBITS_ONE,
				bytesize=serial.EIGHTBITS,
				timeout = 0
			)

			self.ser_EStop.isOpen()
			print("*EStop* - Port successfully Opened.")

		def updateArmingState(self):
			data = self.ser_EStop.readline()                #Read in data from serial port
			
			if(len(data) > 0):                              #
				self.armingState = data[len(data)-1]
				self.noDataCount = 0
			else:
				if(self.noDataCount > 10):              #Watchdog to disarm is no data is received from EStop over 10 cycles
					self.armingState = 0
					self.noDataCount = 0
				else:
					self.noDataCount += 1
			return int(self.armingState)

		def closePort(self):
			self.ser_EStop.close()

								
#####################################
#                Main               #
#####################################
if __name__ == "__main__":
	test = EStop(port = 'COM8', baudrate = 115200)
	while True:
			
	##        print('--------------------')
	##        print(time.perf_counter())
			test.updateArmingState()
			print(test.armingState)
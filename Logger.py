# from enum import enum #FIX

import numpy as np
import csv
import plotly
import plotly.graph_objs as go
import time
import subprocess
import platform
from math import *

from Filter import Filter

class Logger():

	def __init__(self, logFileName='data.csv'):
		self.varDict  = dict()	 	# Create an empty dictionary to store variable names # This list should contain all variables names to log # Alphebatical order is prefered for readability but not
		self.varNames = []	

		self.variableListLocked = False # Determines whether new variables can be added to the dictionary of variables
										# *Note* new variables can only be added to the logger before the first cycle is
										# saved (first called to saveData()). After this the variable dictionary is 'locked'
										# and no new variables can be added. This is to ensure that eadch row of the data 
										# array is always the same size cycle to cycle	 	

		self.fileName = logFileName
		self.timer            = []	# Stores the time at which each control cycle was saved
		self.data             = []
		self.dataCurrentCycle = []  # Stores variable data for 1 control cycle (until)
		
			

	########################################################################
	# Data input must be an array of tuples [(varname, value), 
	# (varname2, value2), ... ]
	# Each variable is polulated in the corresponding place of 
	# dataCurrentCycle using the index map provided by varNames
	########################################################################
	def getData(self, data):
		
		for item in data:
			if not self.variableListLocked and item[0] not in self.varNames: # Variable names are only allowed to be added during the first loop of the logger
				self.varNames.append(item[0])								 # Add the variable name to the variable name list
				self.varDict[item[0]] = len(self.varNames) - 1				 # Add the variable name and unique key index to the dictionary
				self.dataCurrentCycle.append(item[1])						 # Add new variable data to the end of the data list (same index as key)
			else:
				try:
					self.dataCurrentCycle[self.varDict[item[0]]] = item[1] 		 # Populate the current cycle array at the index specified by the variable name
				except:
					print("*LOGGER* - Attempted to log a new variable type: ", item[0], " after the first cycle. Make sure that any variables to be logged are sent to the 'getData' method before calling saveData to save the cycle. New varialbes cannot be logged after the first cycle!")

	########################################################################
	# This method appends 1 cycle of data to the file. It should be called
	# at the end of a cycle when all data has been populated into 
	# self.dataCurrentCycle
	# If any field is unpopulated, it will default to 0
	########################################################################
	def saveData(self):
		# print(self.dataCurrentCycle[0:7])
		self.data.append(list(self.dataCurrentCycle)) # Must make a call to list() to append a copy of the list not a reference to the list
		self.timer.append(time.perf_counter())
		# self.dataCurrentCycle = [0]*len(self.varDict)  #Clear all data to 0 in preparation for the next cycle
		self.variableListLocked = True

	def generatePlots(self, plotName, variablesToPlot=None):
		traces = []
		for name in variablesToPlot:
			print(name, self.varDict[name]);
			tempTrace = go.Scatter(
				x = self.timer,
				y = [col[self.varDict[name]] for col in self.data],
				mode = 'lines',
				name = name
			)

			traces.append(tempTrace)
		if (platform.system() == 'Linux'):
			subprocess.call(['sudo', 'chmod', '777', plotName+".html"])
		plotly.offline.plot(traces, filename=plotName+".html")

	def saveDataToFile(self):
		with open(self.fileName , 'w') as f:
			if (platform.system() == 'Linux'):
				subprocess.call(['sudo', 'chmod', '777', self.fileName])# Open file for appending a new line of data
			writer   = csv.writer(f, delimiter='\t', lineterminator='\n')  # Writer will separate data with tabs
			writer.writerow(self.varNames)									# Write header to first row (list of variable names)

			for cycle in self.data:
				writer.writerow(cycle)		 # Write a row of data to the file

			


#####################################
#      Main during unit testing     #
#####################################
if __name__ == "__main__":
	testLogger = Logger()
	LP = Filter(order=10, cutoff=0.01, channelNum=1)

	x = np.arange(0, 7, 0.01)
	# print(x)
	lastTime = time.perf_counter()
	
	for i in range(len(x)):
		curtime = time.perf_counter()
		originalSignal = sin(x[i])
		inFilter = sin(x[i])+(np.random.rand(1)-0.5)
		outFilter = LP.runFIR(inFilter)
		testLogger.getData([('originalSignal', originalSignal), ('posx', inFilter[0]), ('posxFiltered', outFilter[0]), ('dt', curtime-lastTime)])
		testLogger.saveData()
		lastTime = curtime
		
	testLogger.saveDataToFile()
	testLogger.generatePlots("TEST",['originalSignal','posx','posxFiltered'])
	print('DONE')

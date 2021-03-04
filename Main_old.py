from NatNetClient       import NatNetClient
from PB_Control         import PB_Control
from Trajectory_Planner import Trajectory_Planner
from Sensor             import Sensor
from Cleanflight_MSP    import Cleanflight_MSP
from EStop              import EStop
from threading          import Thread
from threading          import Event
from Logger             import Logger
import struct
import time
import numpy as np


# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, latency, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
    pass
    
# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame(frameID, pos, orient, trackingValid): #NOTE: assign 4 markers to the leader(#1), 5 to copter number 2, 6 to copter number 3, and so on!
    global positions, orientations, trackingFlags, numCopters
    global payloadPose
    if (frameID == 3):
        payloadPose = [pos[0], -pos[2], pos[1], orient[0], orient[1], orient[2], orient[3]]
    else:
        index = frameID - 4
        tempPos = [pos[0], -pos[2], pos[1]] #To transform the camera frame to the inertial frame
        positions[index] = tempPos
        orientations[index] = orient
        trackingFlags[index] = trackingValid
        optitrackThread.callCounter += 1
        if (optitrackThread.callCounter % numCopters == 0):
            event.set()
    
def mainThread_run():
    global positions, orientations, trackingFlags, numCopters, payloadPose #These are the interface variables to the optitrackThread
    global commandsToGo #This is the interface to the comThread
    global ARM, DISARM, ANGLE_MODE, NEUTRAL, ZERO_ROLL, ZERO_PITCH, ZERO_THROTTLE, ZERO_YAW_RATE
    loopCounter = 0
    expTime = 0
    
    while True:
        EStop_failsafe.updateArmingState() #Read data from Estop
        ##Normal closed-loop run in safe mode##
        if(trajPlanner.ARM_FLAG == True and trajPlanner.FAILSAFE_FLAG == False and sensor.FAILSAFE_FLAG == False and EStop_failsafe.armingState == ord('1')):
            event.wait()  #Wait untill the camera measurements are updated for all the drones
            event.clear() #Clear the event for the next cycle
            if (sensor.initFlag == False):
                sensor.process(positions, orientations, trackingFlags)
                expInitTime = time.perf_counter()
            else: ##THIS IS THE MAIN CLOSED_LOOP
                expTime = time.perf_counter() - expInitTime #This is the experiment timer which starts at zero as soon as the experiment is properly initialized.
                sensor.process(positions, orientations, trackingFlags)
                trajPlanner.generate(expTime, sensor.Position, sensor.Velocity)
                controller.control_allocation(expTime, sensor.yawFiltered,
                                              trajPlanner.errors, trajPlanner.phase, trajPlanner.rampUpDuration, trajPlanner.rampDownDuration)
                #Set commandsToGo
                commandsToGoTemp = []
                for i in range(numCopters):
                    commandsToGoTemp.append(controller.mappedCommands[i] + [ARM, ANGLE_MODE, NEUTRAL, NEUTRAL])
                commandsToGo = commandsToGoTemp
                #Log data
                logger.getData([('posPayloadX', payloadPose[0]), ('posPayloadY', payloadPose[1]), ('posPayloadZ', payloadPose[2])])
                logger.getData([('posPayloadQ0', payloadPose[3]), ('posPayloadQ1', payloadPose[4]), ('posPayloadQ2', payloadPose[5]), ('posPayloadQ3', payloadPose[6])])
                logger.getData([('posDesiredX0', trajPlanner.desiredPose[0]), ('posDesiredY0', trajPlanner.desiredPose[1]), ('posDesiredZ0', trajPlanner.desiredPose[2])])
                for i in range (numCopters):
                    logger.getData([('Fx'+str(i), controller.fXYZ[i][0]), ('Fy'+str(i), controller.fXYZ[i][1]), ('Fz'+str(i), controller.fXYZ[i][2])])
                    logger.getData([('posErrX'+str(i), trajPlanner.errors[i][0]), ('posErrY'+str(i), trajPlanner.errors[i][1]), ('posErrZ'+str(i), trajPlanner.errors[i][2])])
                    logger.getData([('posx'+str(i), sensor.Position[i][0]), ('posy'+str(i), sensor.Position[i][1]), ('posz'+str(i), sensor.Position[i][2])])
                    logger.getData([('velx'+str(i), sensor.Velocity[i][0]), ('vely'+str(i), sensor.Velocity[i][1]), ('velz'+str(i), sensor.Velocity[i][2])])
                    logger.getData([('yaw'+ str(i), sensor.yawFiltered[i])])
                    logger.getData([('rollCmd'+str(i), controller.roll[i]),('pitchCmd'+str(i), controller.pitch[i]),
                                    ('throttleCmd'+str(i), controller.throttle[i]), ('yawRateCmd'+str(i), controller.yawRate[i])])
                    logger.getData([('mspRoll'+str(i), controller.mappedCommands[i][0]), ('mspPitch'+str(i), controller.mappedCommands[i][1]),
                                    ('mspThrottle'+str(i), controller.mappedCommands[i][2]), ('mspYawRate'+str(i), controller.mappedCommands[i][3])])
                    logger.getData([('trackingFlag'+str(i), trackingFlags[i])])
                logger.saveData()
        else:
            #Case1: Experiment completed
            if(trajPlanner.ARM_FLAG == False and trajPlanner.FAILSAFE_FLAG == False and sensor.FAILSAFE_FLAG == False and EStop_failsafe.armingState == ord('1')):
                print("Experiment completed successfully.")
            #Case2: Failsafe triggered 
            else:
                if (EStop_failsafe.armingState != ord('1')):
                    print("Failsafe, root cause: stop button")
                elif(sensor.FAILSAFE_FLAG == True):
                    print("Failsafe, root cause: camera system lost track of at least one copter")
                else:
                    print("Failsafe, root cause: large deviation from the virtual points")
            #Send disarm commands to all copters
            commandsToGoTemp = []
            for i in range (numCopters):
                commandsToGoTemp.append([ZERO_ROLL, ZERO_PITCH, ZERO_THROTTLE, ZERO_YAW_RATE, DISARM, ANGLE_MODE, NEUTRAL, NEUTRAL])
            commandsToGo = commandsToGoTemp
            #Saving data to file and generating plots
            logger.saveDataToFile()
            logger.generatePlots("PayLoad_Position",['posPayloadX', 'posPayloadY', 'posPayloadZ'])
            logger.generatePlots("Desired_Position_Copter0",['posDesiredX0','posDesiredY0','posDesiredZ0'])
            logger.generatePlots("Yaw_Orientations",['yaw'+str(i) for i in range (numCopters)])
            logger.generatePlots("Tracking_Flags",['trackingFlag'+str(i) for i in range (numCopters)])
            for i in range (numCopters):
                logger.generatePlots("High-level_Force_Commands"+str(i),['Fx'+str(i),'Fy'+str(i),'Fz'+str(i)])
                logger.generatePlots("Position_Errors_Copter"+str(i),['posErrX'+str(i),'posErrY'+str(i),'posErrZ'+str(i)])
                logger.generatePlots("Position_Copter"+str(i),['posx'+str(i),'posy'+str(i),'posz'+str(i)])
                logger.generatePlots("Velocity_Copter"+str(i),['velx'+str(i),'vely'+str(i),'velz'+str(i)])
                logger.generatePlots("Reference_Commands_Copter"+str(i),['rollCmd'+str(i),'pitchCmd'+str(i),'throttleCmd'+str(i),'yawRateCmd'+str(i)])
                logger.generatePlots("MSP_Commands_Copter"+str(i),['mspRoll'+str(i),'mspPitch'+str(i),'mspThrottle'+str(i),'mspYawRate'+str(i)])
                debugLogger.generatePlots("Debug_MSP_Commands_Copter"+str(i),['dmspRoll'+str(i),'dmspPitch'+str(i),'dmspThrottle'+str(i),'dmspYawRate'+str(i)])
            break
        loopCounter += 1
        if (loopCounter%1000 == 0):
            print('Average loop rate is:',loopCounter/(time.perf_counter() - expInitTime),'Hz')

def comThread_run():
    global numCopters
    global ARM, DISARM, ANGLE_MODE, NEUTRAL, ZERO_ROLL, ZERO_PITCH, ZERO_THROTTLE, ZERO_YAW_RATE
    HZ = 100 #This thread runs at 100 HZ
    ## Arming procedure:
    # 1) Start sending MSP RX with all sticks in middle position (1550) and throttle stick down (1050) & DISARM #We must send a disarm
    # 2) Start sending MSP RX with all sticks in middle position (1550) and throttle stick down (1050) & ARM
    disArmCommandsToGoMSP = [ZERO_ROLL, ZERO_PITCH, ZERO_THROTTLE, ZERO_YAW_RATE, DISARM, ANGLE_MODE, NEUTRAL, NEUTRAL] # The 1600 is to enable the "Angle Mode" in clean flight.
    armCommandsToGoMSP    = [ZERO_ROLL, ZERO_PITCH, ZERO_THROTTLE, ZERO_YAW_RATE,    ARM, ANGLE_MODE, NEUTRAL, NEUTRAL]
    dataLength = len(disArmCommandsToGoMSP)
    dataBytes = 2*dataLength
    direction = '<'
    h = 'h'
    for i in range(1, 20):  #command for about a second before arming
        for i in range (numCopters):
            eval('cleanflightMSP'+str(i)+'.sendMSP(direction, dataBytes, 200, disArmCommandsToGoMSP, direction+ str(dataLength) + h)')
        time.sleep(1/HZ)
    for i in range(1, 20):
        for i in range (numCopters):
            eval('cleanflightMSP'+str(i)+'.sendMSP(direction, dataBytes, 200,    armCommandsToGoMSP, direction+ str(dataLength) + h)')
        time.sleep(1/HZ)
    ## Continuously sending the latest available commands to each copter    
    while True:
        for i in range (numCopters):
            debugLogger.getData([('dmspRoll'+str(i), commandsToGo[i][0]), ('dmspPitch'+str(i), commandsToGo[i][1]),
                                    ('dmspThrottle'+str(i), commandsToGo[i][2]), ('dmspYawRate'+str(i), commandsToGo[i][3])])
        debugLogger.saveData()
        for i in range (numCopters):
            eval('cleanflightMSP'+str(i)+'.sendMSP(direction, dataBytes, 200,    commandsToGo[i], direction+ str(dataLength) + h)')
        time.sleep(1/HZ)
   
############
##  MAIN  ##
############
if (__name__ == '__main__'):
    numCopters = 1
    positions = []
    orientations = []
    trackingFlags = []
    payloadPose = [0, 0, 0, 0, 0, 0, 0]
    ARM = 1600; DISARM = 1000; ANGLE_MODE = 1600; NEUTRAL = 1000;
    ZERO_ROLL = 1500; ZERO_PITCH = 1500; ZERO_YAW_RATE = 1500; ZERO_THROTTLE = 1000;
    zeroCommands = [ZERO_ROLL, ZERO_PITCH, ZERO_THROTTLE, ZERO_YAW_RATE, ARM, ANGLE_MODE, NEUTRAL, NEUTRAL]
    commandsToGo = [] # This is a list of lists. Each list contains the low-level commands for each copter in the order of copter IDs.
    for i in range (numCopters):
        commandsToGo.append(zeroCommands) #Roll, pitch, throttle, yaw rate, aux1, aux2, ...
        positions.append([])
        orientations.append([])
        trackingFlags.append(False)
    initTime = 0.0
    expTime = 0.0

    ######## Creating instances of all required classes (creating objects) #########
    ################################################################################
    event = Event() # Event object to sync the main thread and the optitrack thread

    #To run in the optitrackThread
    optitrackThread                   = NatNetClient()        # This will create a new NatNet client to connect to motive
    optitrackThread.newFrameListener  = receiveNewFrame       # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    optitrackThread.rigidBodyListener = receiveRigidBodyFrame

    #To run in the mainThread
    sensor         = Sensor(numCopters)                       #Sensor object. Grabs camera measurements and estimates linear velocities.
    trajPlanner    = Trajectory_Planner()                     #Trajectory planning object. Generates time dependent trajectories or set points.
    controller     = PB_Control()                             #Passivity based controller object. Determines desired thrust, roll, and pitch of each copter.
    EStop_failsafe = EStop('COM6', 115200)                   #EStop object. When pressed, EStop disarms FC & puts in failsafe mode.
    logger         = Logger()                                 #Loggs and plots variables
    debugLogger         = Logger()                                 #Loggs and plots variables

    #To run in the comThread
    cleanflightMSP0 = Cleanflight_MSP('COM10', 115200)          #MSP object to comunicate with the head copter. Packages messages in the MSP Protocal.
    #cleanflightMSP1 = Cleanflight_MSP('COM5', 115200)            #MSP object. Packages messages in the MSP Protocal.
    #cleanflightMSP2 = Cleanflight_MSP('COM11', 115200)          #MSP object. Packages messages in the MSP Protocal.

    time.sleep(1)

    ######## Creating and running all the three threads #######
    ##############################################
    optitrackThread.run()                       #Start up the streaming client now that the callbacks are set up. This will run perpetually, and operate on a separate thread.
    print("Comunication with cameras established. (Thread #1)")

    comThread = Thread(target = comThread_run)  #Thread to communicate with the copters. (Send commands only)
    comThread.start()                           #Start up thread to constantly send rx data
    print("Comunication with copters established and copters are armed. (Thread #2)")
    time.sleep(1.5)                               #Wait for 1.5 second to let the copters go through the arming procedure.

    mainThread = Thread(target = mainThread_run)#The main thread which runs sensor, trajectory planner, and controller modules.
    mainThread.start()                          #Start up thread to close the feed-back control loop
    print("Main thread initiated to start the experiment. (Thread #3)")
    

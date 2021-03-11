from NatNetClient       import NatNetClient
from Sensor             import Sensor
from Logger             import Logger
from threading          import Thread
from threading          import Event
import time

# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
def receiveNewFrame(frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged):
    pass

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame(frameID, pos, orient,
                          trackingValid):  # NOTE: assign 4 markers to the leader(#1), 5 to copter number 2, 6 to copter number 3, and so on!
    global positions, orientations, trackingFlags, RigidBodyCount
    global payloadPose
    index = frameID - 3
    tempPos = [pos[0], -pos[2], pos[1]]  # To transform the camera frame to the inertial frame
    positions[index] = tempPos
    orientations[index] = orient
    trackingFlags[index] = trackingValid
    optitrackThread.callCounter += 1
    if (optitrackThread.callCounter % RigidBodyCount == 0):
        event.set()

def mainThread_run():
    global positions, orientations, trackingFlags, RigidBodyCount, payloadPose #These are the interface variables to the optitrackThread
    loopCounter = 0
    expTime = 0
    try:
        while (True):
            if (sensor.FAILSAFE_FLAG == False):
                event.wait()  # Wait untill the camera measurements are updated for all the drones
                event.clear()  # Clear the event for the next cycle
                if (sensor.initFlag == False):
                    sensor.process(positions, orientations, trackingFlags)
                    expInitTime = time.perf_counter()
                else:  ##THIS IS THE MAIN CLOSED_LOOP
                    expTime = time.perf_counter() - expInitTime  # This is the experiment timer which starts at zero as soon as the experiment is properly initialized.
                    sensor.process(positions, orientations, trackingFlags)
                    for i in range(RigidBodyCount):
                        logger.getData([('posx' + str(i), sensor.Position[i][0]), ('posy' + str(i), sensor.Position[i][1]),
                                        ('posz' + str(i), sensor.Position[i][2])])
                        # logger.getData([('velx' + str(i), sensor.Velocity[i][0]), ('vely' + str(i), sensor.Velocity[i][1]),
                        #                 ('velz' + str(i), sensor.Velocity[i][2])])
                        # logger.getData([('yaw' + str(i), sensor.yawFiltered[i])])
                        # logger.getData([('trackingFlag' + str(i), trackingFlags[i])])
                    logger.saveData()

            else:
                print("sensor.FAILSAFE_FLAG is not False anymore")
                time.sleep(1)
                logger.saveDataToFile()
                logger.generatePlots("Yaw_Orientations", ['yaw' + str(i) for i in range(RigidBodyCount)])
                logger.generatePlots("Tracking_Flags", ['trackingFlag' + str(i) for i in range(RigidBodyCount)])
                for i in range(RigidBodyCount):
                    logger.generatePlots("Position_Errors_Copter" + str(i),
                                         ['posErrX' + str(i), 'posErrY' + str(i), 'posErrZ' + str(i)])
                    logger.generatePlots("Position_Copter" + str(i),
                                         ['posx' + str(i), 'posy' + str(i), 'posz' + str(i)])
                    logger.generatePlots("Velocity_Copter" + str(i),
                                         ['velx' + str(i), 'vely' + str(i), 'velz' + str(i)])
                break
            loopCounter += 1
            if (loopCounter % 1000 == 0):
                print('Average loop rate is:', loopCounter / (time.perf_counter() - expInitTime), 'Hz')
    except KeyboardInterrupt:
        time.sleep(0.5)
        print("Finishing the program")
        time.sleep(0.5)
        logger.saveDataToFile()
        logger.generatePlots("Yaw_Orientations", ['yaw' + str(i) for i in range(RigidBodyCount)])
        logger.generatePlots("Tracking_Flags", ['trackingFlag' + str(i) for i in range(RigidBodyCount)])
        for i in range(RigidBodyCount):
            logger.generatePlots("Position" + str(i), ['posx' + str(i), 'posy' + str(i), 'posz' + str(i)])
            logger.generatePlots("Velocity" + str(i), ['velx' + str(i), 'vely' + str(i), 'velz' + str(i)])









if (__name__ == '__main__'):
    RigidBodyCount = 3
    positions = []
    orientations = []
    trackingFlags = []
    for i in range (RigidBodyCount):
        positions.append([])
        orientations.append([])
        trackingFlags.append(False)

    expTime = 0.0
    event = Event() # Event object to sync the main thread and the optitrack thread
    optitrackThread = NatNetClient(ver=(2, 9, 0, 0),
                                   quiet=True)  # This will create a new NatNet client to connect to motive
    optitrackThread.newFrameListener = receiveNewFrame  # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    optitrackThread.rigidBodyListener = receiveRigidBodyFrame
    sensor = Sensor(RigidBodyCount)  # Sensor object. Grabs camera measurements and estimates linear velocities.
    logger = Logger()  # Loggs and plots variables
    debugLogger = Logger()  # Loggs and plots variables
    time.sleep(1)
    optitrackThread.run()  # Start up the streaming client now that the callbacks are set up. This will run perpetually, and operate on a separate thread.
    print("Comunication with cameras established. (Thread #1)")

    mainThread = Thread(target=mainThread_run)  # The main thread which runs sensor, trajectory planner, and controller modules.
    mainThread.start()  # Start up thread to close the feed-back control loop
    print("Main thread initiated to start the experiment. (Thread #3)")
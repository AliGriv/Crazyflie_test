import time
from threading          import Thread
import cflib
from cflib.crazyflie    import Crazyflie



def mainThread_run():
    global commandsToGo
    loopCounter = 0
    while (loopCounter < 20000):
        commandsToGoTemp = []
        for i in range(numCopters):
            # commandsToGoTemp.append([0, 0, loopCounter, 0])
            commandsToGoTemp.append([0, 0, 0, 0])
        commandsToGo = commandsToGoTemp
        loopCounter = loopCounter + 1
        time.sleep(0.001)




def comThread_run():
    # comments for future:
    # consider the time delays, something like arming procedures
    global numCopters
    HZ = 100
    loop_counter = 0;
    com_thread_init_time = 0.0
    while True:
        # for i in range(numCopters):
        #     debugLogger.getData([('dmspRoll' + str(i), commandsToGo[i][0]), ('dmspPitch' + str(i), commandsToGo[i][1]),
        #                          ('dmspThrottle' + str(i), commandsToGo[i][2]),
        #                          ('dmspYawRate' + str(i), commandsToGo[i][3])])
        # debugLogger.saveData()
        if (loop_counter == 0):
            com_thread_init_time = time.perf_counter()
        if (True):

            for i in range(numCopters):
                if (le[i].is_connected):
                    eval('le['+str(i)+']._send_commands(commandsToGo[i])')
                    print(commandsToGo[i])
                else:
                    print("One if the copters not connected anymore")
                    break
            time.sleep(1/HZ)
        else:
            break
        loop_counter = loop_counter + 1
        if (loop_counter%100 == 0):
            print('Average com thread loop rate is:',loop_counter/(time.perf_counter() - com_thread_init_time),'Hz')

class crazy_command:
    """Example that connects to a Crazyflie and send command to the motors and
    the disconnects"""
    global commandsToGo
    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)
        self.is_connected = True
        print('Connecting to %s' % link_uri)
        self._cf.commander.send_setpoint(0, 0, 0, 0)
    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        # Thread(target=self._command_motors).start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

    def _command_motors(self):
        # Unlock startup thrust protection
        roll = commandsToGo[0][0]
        pitch = commandsToGo[0][1]
        yawrate = commandsToGo[0][3]
        thrust = commandsToGo[0][2]
        # print(thrust)
        # self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        #time.sleep(0.1)
        #self._cf.close_link()

    def _send_commands(self, commands):
        # roll = commandsToGo[0][0]
        # pitch = commandsToGo[0][1]
        # yawrate = commandsToGo[0][3]
        # thrust = commandsToGo[0][2]
        self._cf.commander.send_setpoint(commands[0], commands[1], commands[3] ,commands[2])
    def _close_it(self):
        self._cf.close_link()
        self.is_connected = False

if (__name__ == '__main__'):
    numCopters = 1
    commandsToGo = []
    for i in range(numCopters):
        commandsToGo.append([0,0,0,0])
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    le = []
    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    for i in available:
        print(i[0])

    if len(available) > 0:
        for i in range(len(available)):
            le.append(crazy_command(available[i][0]))
        comThread = Thread(target=comThread_run)  # Thread to communicate with the copters. (Send commands only)
        comThread.start()
    else:
        print('No Crazyflies found, cannot run example')

    # Thread(target = CF_command).start()

    mainThread = Thread(target=mainThread_run)  # The main thread which runs sensor, trajectory planner, and controller modules.
    mainThread.start()  # Start up thread to close the feed-back control loop
    print("Main thread initiated to start the experiment. (Thread #3)")
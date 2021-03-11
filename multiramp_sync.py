import threading
import time
from collections import namedtuple
from queue import Queue

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncLogger import SyncLogger


uris = [
    'radio://0/80/2M/E7E7E7E7E3',  # cf_id 0, startup position [-0.5, -0.5]
    'radio://0/80/2M/E7E7E7E7E6',  # cf_id 1, startup position [ 0, 0]
    # Add more URIs if you want more copters in the swarm
]

is_running = 0

class Command:
    id = 0
    roll = 0
    pitch = 0
    yaw = 0
    thrust = 0
    def __init__(self, index):
        self.id = index
    def setCommands(self, roll_val, pitch_val, yaw_val, thrust_val):
        self.roll = roll_val
        self.pitch = pitch_val
        self.yaw = yaw_val
        self.thrust = thrust_val


Commands_list = []
for i in range (len(uris)):
    Commands_list.append(Command(i))

def crazyflie_control(scf):
    global is_running
    cf = scf.cf
    control = controlQueues[uris.index(cf.link_uri)]

    commander = scf.cf.commander


    while True:
        if (is_running):
            command = control.get()
            commander.send_setpoint(command.roll, command.pitch, command.yaw, command.thrust)
            print(command.thrust)
        else:
            return


def control_thread():
    global is_running
    thrust_mult = 1
    thrust_step = 50
    thrust = 20000
    pitch = 0
    roll = 0
    yaw = 0
    is_running = 1
    for i in range(len(uris)):
        cf_id = i
        Commands_list[i].setCommands(0, 0, 0, 0)
        command = Commands_list[i]
        print(' - Zero Running: {} on {}'.format(command, cf_id))
        controlQueues[cf_id].put(command)
    # time.sleep(0.01)
    while thrust >= 20000:

        for i in range(len(uris)):
            cf_id = i
            Commands_list[i].setCommands(roll, pitch, yaw, thrust)
            command = Commands_list[i]
            print(' - Running: {} on {}'.format(command, cf_id))
            controlQueues[cf_id].put(command)
        if thrust >= 25000:
            thrust_mult = -1
        thrust += thrust_step * thrust_mult
        # time.sleep(0.01)

    for ctrl in controlQueues:
        Commands_list[i].setCommands(0, 0, 0, 0)
        command = Commands_list[i]
        ctrl.put(command)
    is_running = 0




if __name__ == '__main__':
    controlQueues = [Queue() for _ in range(len(uris))]

    cflib.crtp.init_drivers(enable_debug_driver=False)
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        # swarm.parallel_safe(activate_high_level_commander)
        # swarm.parallel_safe(reset_estimator)

        print('Starting multi ramp!')

        threading.Thread(target=control_thread).start()

        swarm.parallel_safe(crazyflie_control)

        time.sleep(1)

        swarm.close_links()
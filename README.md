# Crazyflie_test
This is a project to fly crazyflie drone(s) autonomously, using Optitrack Motive motion capture system.
The code is meant to be run on Ubuntu 16.04. However, using NatNetClient_Old.py and corrections to "COM" port names in the Main.py should lead to the correct version for Windows-based usage.
The project is currently under development and probably has not been cleaned or a few bugs may still exist. The control algorithm is based on [Keyvan Mohammadi's paper](https://ieeexplore.ieee.org/abstract/document/8593952). 
## Prequisites
1. [Crazyflie Python Library](https://github.com/bitcraze/crazyflie-lib-python) 
2. SciPy library
3. Plotly [Python Library](https://plotly.com/python/])
4. Setup Ethernet connection using LAN cable between Ubuntu and Windows (which has Motive). See [this video](https://youtu.be/5e0sMf48cBk).
## Other useful notes:
1. To run Main.py on Ubuntu, use the current NatNetClient.py and pay specific attention to the NatNet Version. For example, with our device we had to use version 2.9.0.0.
``` optitrackThread                   = NatNetClient(ver=(2, 9, 0, 0), quiet=True) ```
2. In our setup, an Emergency-Stop  button was used which was defined in ```EStop_failsafe = EStop('/dev/ttyUSB0', 115200)```. In windows, it will be defined as ```EStop_failsafe = EStop('COM5', 115200) ```

3. In our setup, for experimental purposes the first rigid-body (or the one with three marker) is assumed to be the payload, at least for now. Please see the definition of ```receiveRigidBodyFrame``` function.
4. The callback function is different for Windows-based code. Use the following in your Main.py:
```
def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, latency, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
    pass
```

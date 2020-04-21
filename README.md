# The bluetooth_bridge package

This package utilizes bidirectional communication to facilitate serial communication of ROS messages between two workstations.
This enables multiagent systems to relay information when in close proximity without having to rely on a centralized communication network that may suffer from reliability issues.

This package is indended to be used with bluetooth devices, but is compartimentalized such that any communication device that can connect to a virtual linux serial port should work.
While using other technologies may increase operational distance between agents, bandwidth may be reduced.

Advantages of this package:
   - low latency
   - high reliability
   - scalable to multiple connections*
   - low computational overhead


There are two main libraries in this package. These are described below, but please check each package README for more details.

## serial_bridge 
This is a ROS oriented bridge for full-message serial communication between two PCs over a serial port and is based on the `rosserial` protocol. 

Note that `bluetooth_bridge` differs from standard `rosserial` packages which:
   - Only allow for unidirectional communication 
   - Assume the serial device is a microcontroller
   - Require the generation of a modified `ros_lib` to compile against

As such, this package is not a serial `client` or a serial `server`, it a bridge between a shared virtual serial port on two Linux PCs.

## BlueToothPortManager
This accesses Bluetooth Hardware intefaces to scan and detect known agents and negotiate the creation of a virtual serial port to be used by `serial_bridge`. 
Note that this `Manager` also avoids the exclusivity of being either a `client` or a `server`. The `Manager` can be both a `client` and `server` to a number of other agents. In theory, this is only limited by the number of virtual ports that can be created (~30 on Ubuntu).

With the `Manager` acting as a negotiator, the resolution and hand-off of agent-local information to nearby agents via `serial_bridge` is entirely autonomous.

## Project Status
Stable. Working and tested with between two ROS Masters as long as the data shared is within the bandwidth limitations of Bluetooth.

### What's next
 - The basis for multiple simultaneous serial bridge connections are in place, but needs further development and testing
 - Documentation
 - Reworking `BlueToothPortManager` to create Serial objects and pass them to `serial_bridge` rather than relying on virtual ports
 - Better error handling

# Installation
Python3.5 is required for this project. It does not need to be your default python version, but it does need to be installed.

## Install necessary packages
Install bluetooth C libraries:

    $ sudo apt-get install libbluetooth-dev bluez bluez-tools

and install python bluetooth wrappers and utilities

    $ sudo pip3 install pygatt pygattlib pybluez

note that if you only have python 3.5 installed, you can just use pip- no need for pip3.

and add your user to the `lp` and `netdev` groups:

    $ sudo usermod -aG lp $USER && sudo usermod -aG netdev $USER

### For users of Ubuntu 16.04
You will need to start the Bluez service in compatibility mode for it to behave as expected. Edit the service spec:

    $ sudo vim /etc/systemd/system/dbus-org.bluez.service

change the 

    ExecStart=/usr/lib/bluetooth/bluetoothd

to 

    ExecStart=/usr/lib/bluetooth/bluetoothd --compat

then run:

    $ systemctl daemon-reload && systemctl restart bluetooth && chmod 777 /var/run/sdp

### Setup complete
The local system is now ready to setup virtual serial ports over bluetooth. Repeat this installation on any remote machines that you wish to connect to via serial.


# Running the project
This is subject to change

## BluetoothPortManager
Execute with sudo priviledges in the subpackage root:

    $ sudo ./port_manager.py

## serial_bridge
`serial_bridge` is a ROS node and should be invoked through the ROS systems:

    $ roslaunch serial_bridge serial_bridge.launch

Note that the launch file above has arguments that may need to be changed for your particular use case.



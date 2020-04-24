
# The bluetooth_port_manager package

This package is a python-based virtual port manager over bluetooth communication. In order to use this package, your machines must have an accessible bluetooth interface at the system level.

This library, when running on two machines with bluetooth interfaces, can autonomously agree on the creation of a virtual serial port between the two machines. The only requirement is that each machine must have the other machine's bluetooth device MAC Address on their device-whitelist. A whitelist is used as a safety measure to only allow connections to/from devices known to be safe.

## Installation
This package requires Python3.5 or greater. Please ensure you have Python3.5 installed.
Additionally, install these required packages via `pip`

   $ pip install pyyaml pygatt pygattlib pybluez


In order to set permissions to give python access to system level bluetooth control, set permissions with: 

 $ sudo setcap 'cap_net_raw,cap_net_admin+eip' /usr/bin/python3.5

NOTE: the above is necessary when creating serial port objects. Since the utilization of system calls, running the `Manager` with sudo is required anyway.

Finally, install the library locally. In the root of the project run

   $ pip install -e . --no-deps

## How to Use

### Fill in the whitelist
The whitelist is a yaml file with that consists of MAC addresses and `nice` values for known safe bluetooth devices. In order for the `Manager` to create a virtual serial port, remote MAC Addresses must be added to the whitelist.

#### MAC Address
This is the MAC address of the remote that you are clearing as safe to connect to the local machine. You can find the MAC address of the remote by running
    
    $ hcitool dev

on the remote machine and selecting the MAC address of the bluetooth device used by default.

#### Nice values
The `nice` value is a required field and is what determines whether or not the `Manager` acts a `host` or a `client` to a remote agent. Nice values must follow these two rules:
   - No two `nice` values can be the same
   - the nice values for a given MAC Address should be the same across all possible remotes

#### hostname
This is mostly required for debugging and convenience and represents the expected hostname of the machine the bluetooth device is a part of.

## Running the port manager
You can run the port manager by simply executing it with 

    $ sudo ./port_manager.py

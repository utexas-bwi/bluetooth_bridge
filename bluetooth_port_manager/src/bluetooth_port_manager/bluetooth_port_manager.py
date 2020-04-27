#! /usr/bin/env python3.5

import argparse
import bluetooth
from bluetooth.ble import DiscoveryService
import copy
import math
import os
import subprocess
import sys
import threading
import time
import yaml

__author__ = "maxsvetlik@utexas.edu (Max Svetlik)"

class BluetoothPortManager:
    """
    Main class providing interface that polls for bluetooth devices that are whitelisted on local configuration files.

    BluetoothPortManager periodically polls for high energy bluetooth devices, while `stale` addresses after a timeout period.
    This class can attempt to make serial connections to bluetooth devices over virtual Linux ports over system calls, or over
    BluetoothSerial objects provided by the pybluez library.

    Parameters:
    legacy_port (bool): if True, serial connections will be made over virtual Linux ports using system calls
    poll_frequency (int): how often the polling update loop is run, in seconds
    scan_timeout (int): how long to search for bluetooth devices on each scan, in seconds
    address_timeout (int): a bluetooth device will be considered stale after a drop in communication for this time, in seconds
    whitelist_path (string): the path to the yaml file describing what devices are acceptable to connect to

    """

    def __init__(self, legacy_port=False, poll_frequency = 1, scan_timeout=4, address_timeout=20, whitelist_path=''):
        self.legacy_port = legacy_port
        self.poll_frequency = poll_frequency #seconds
        self.scan_timeout = scan_timeout #seconds
       	self.address_timeout = address_timeout #seconds; time, after which a detected mac address is considered 'stale'

        full_path = whitelist_path
        if whitelist_path == '':
            package_directory = os.path.dirname(os.path.abspath(__file__))
            full_path = package_directory+"/config/whitelist.yaml"
        try:
            self.mac_whitelist = self._load_yaml(full_path)
        except:
            print("Could not load whitelist yaml file.")
            exit(-1)

        self._setup_args()
        self.run_thread = threading.Thread(target=self._run,)

        self.clients = [] #who we're serving
        self.servers = [] #who we're connected to
        self.serial_clients = dict()
        self.serial_servers = dict()

        self.stale_sockets = dict() #sockets that have not been seen in self.address_timeout seconds
        self.available_devices = dict() #BLE scan results and the last time they were seen
        self.available_devices_lock = threading.Lock()
        self.shutdown = False

        self.my_mac = bluetooth.read_local_bdaddr()[0]
        self.my_nice = self.mac_whitelist[self.my_mac]['nice']
        self.le_devices = []
        self.std_devices = []

    def _setup_args(self):
        """ Helper function to set up Logger Levels """

        # Logger levels
        self.DEBUG = False
        self.DEBUGG = False
        self.DEBUGGG = False

        parser = argparse.ArgumentParser()
        parser.add_argument('--verbose', '-v', action='count')
        args = vars(parser.parse_args(sys.argv[4:]))
        verbosity_count = args['verbose']

        if verbosity_count is None:
           return
        if verbosity_count >= 1:
           self.DEBUG = True
        if verbosity_count >= 2:
           self.DEBUGG = True
        if verbosity_count >= 3:
           self.DEBUGGG = True

    def _sum_mac_addr(self, mac_addr):
        """ Deprecated. Helper function to set connection policies """

        acc = 0
        for byte_pair in mac_addr.split(':'):
            acc += int(byte_pair[0], 16)
            acc += int(byte_pair[1], 16)
        return acc

    def _load_yaml(self, yaml_path):
        """ Helper function to load whitelist file """

        with open(yaml_path, 'r') as stream:
            return yaml.safe_load(stream)

    def get_client_serial_ports(self):
        """ Returns connected clients """

        ret = []
        for name, port in self.serial_clients.items():
            ret.append(port)
        for name, tup in self.serial_servers.items():
            ret.append(tup[1])
        return ret

    def start(self):
        """ Begins the main loop on a thread """

        self.run_thread.start()

    def stop(self):
        """ Initiates shutdown proceedures and waits for main loop to exit """

        self.shutdown = True
        print("Waiting for process to stop...")
        self.run_thread.join()
        print("Done.")

    def _run(self):
        """ Main loop """

        # Setup address polling
        threading.Timer(self.poll_frequency, self.scan_addresses).start()
        #threading.Timer(self.poll_frequency, self.scan_addresses_low_energy).start()
        threading.Timer(self.poll_frequency, self.scan_addresses_std).start()

        try:
            while not self.shutdown:
                availble_devices = dict()

                with self.available_devices_lock:
                    available_devices = copy.deepcopy(self.available_devices)

                for addr in available_devices.keys():
                    if addr in self.mac_whitelist.keys():
                        if self.DEBUGGG:
                            print("Remote " + addr + " found in whitelist.")
                        remote_nice = self.mac_whitelist[addr]['nice']

                        # standard policy, greater nice value acts as server
                        if self.my_nice > remote_nice:
                            #spin up server
                            if self.legacy_port and  addr not in self.clients:
                                self.clients.append(addr)
                                self.rfcomm_serve()
                            elif not self.legacy_port and addr not in self.serial_servers.keys():
                                if self.DEBUGG:
                                    print("Found remote with smaller nice value. Hosting...")
                                self.bt_serial_object_server(self.my_nice)

                        else:
                            #spin up client
                            if self.legacy_port and addr not in self.servers:
                                self.servers.append(addr)
                                self.rfcomm_client(addr)
                            elif not self.legacy_port and addr not in self.serial_clients.keys():
                                if self.DEBUGG:
                                    print("Found remote with greater nice value. Connecting...")
                                self.bt_serial_object_client(addr, remote_nice)
                time.sleep(0.5)

        except:
            print("Error in executing BluetoothPortManager main loop. Shutting down")
            self.shutdown = True

    def scan_addresses(self):
        """ Polling loop that aggregates and updates devices according to scans and removal policy """

        le_devices = self.le_devices
        std_devices = self.std_devices
        devices = le_devices + std_devices
        with self.available_devices_lock:
            addrs = copy.deepcopy(self.available_devices)

        for address in devices:
            addrs[address] = math.ceil(time.time())

        if self.DEBUGGG:
            print("Detected devices:")
            for item, timestamp in addrs.items():
                print("{} ({})".format(item, timestamp))

        #remove stale addresses
        for k in list(addrs.keys()):
            if time.time() - addrs[k] > self.address_timeout:
                if k in self.serial_clients.keys():
                    print("Possible stale socket to " + k + " due to inactivity timeout.")
                    self.stale_sockets[self.serial_clients[k]] = k
                elif k in self.serial_servers.keys():
                    print("Possible stale socket detected to " + k + " due to inactivity timeout.")
                    tup = self.serial_servers[k]

        with self.available_devices_lock:
            self.available_devices = addrs

        if not self.shutdown:
            threading.Timer(self.poll_frequency, self.scan_addresses).start()

    def scan_addresses_low_energy(self):
        """ Initiates a scan for bluetooth LE hardware """

        service = DiscoveryService()
        devices = []
        #print("Scanning low energy")
        for addr, name in service.discover(self.scan_timeout).items():
            devices.append(addr)
        with self.available_devices_lock:
            self.le_devices = devices
        if not self.shutdown:
            threading.Timer(self.poll_frequency, self.scan_addresses_low_energy).start()

    def scan_addresses_std(self):
        """ Initiates a scan for 'classic' bluetooth devices, such as those on Desktop computers or laptops """

        ret = []
        try:
            ret = bluetooth.discover_devices(lookup_names=False, duration=self.scan_timeout)
        except:
            #TODO throw err
            print("Error connecting to local bluetooth adapter. Unable to scan.")
        with self.available_devices_lock:
            self.std_devices = ret
        if not self.shutdown:
            threading.Timer(self.poll_frequency, self.scan_addresses_std).start()

    def bt_add_channel(self, channel_no):
        """ Legacy function. Adds a connection channel profile for use with virtual Linux port """

        command = ['sdptool', 'add', '--channel=22', 'SP']
        result = subprocess.run(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        if result.returncode != 0:
            print("Unable to add channel via sdptool.")
            #TODO throw error
            pass

    def rfcomm_serve(self):
        """ Legacy function. Host a connection over a virtual Linux port """

        self.bt_add_channel(22)
        command = ['rfcomm', 'watch', '/dev/rfcomm0', '22']
        result = subprocess.run(command, stdout=subprocess.DEVNULL)

    def rfcomm_client(self, server_mac):
        """ Legacy function. Connect to a host over a virtual Linux port """

        self.bt_add_channel(22)
        command = ['rfcomm', 'connect', '/dev/rfcomm0', str(server_mac), '22']
        result = subprocess.run(command, stdout=subprocess.DEVNULL)
        self.servers.remove(server_mac)

    # host a serial socket
    def bt_serial_object_server(self, port):
        """ Host a connection over a serial object """

        server_sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )

        #TODO
        server_sock.bind(("",port))
        #TODO
        server_sock.listen(1)

        client_sock,address = server_sock.accept()
        print("Accepted connection from ",address[0])

        self.serial_servers[address[0]] = (server_sock, client_sock)
        with self.available_devices_lock:
            self.available_devices[address[0]] = math.ceil(time.time())

    # connect to a remote socket
    def bt_serial_object_client(self, mac_addr, port):
        """ Connect to a host over a serial object """

        sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )
        try:
            sock.connect((mac_addr, port))
            self.serial_clients[mac_addr] = sock
            with self.available_devices_lock:
                self.available_devices[mac_addr] = math.ceil(time.time())
            print("Connected to " + mac_addr)
        except:
            print("Connection to " + mac_addr + " unsuccessful. Retrying.")

    def get_stale_sockets(self):
        """ Return list of stale sockets """

        return self.stale_sockets

    #TODO is this right?
    def remove_socket(self, my_socket):
        """ Removes a socket from the Manager's resource list. Needed if a connection is closed manually """

        for mac, socket in self.serial_clients.items():
            if my_socket == socket:
                del self.serial_clients[mac]

        for mac, (socket_s, socket_c) in self.serial_servers.items():
            if socket_s == socket or socket_c == my_socket:
                del self.serial_servers[mac]

    def remove_stale_socket(self, socket):
        """ Removes a socket according to removal policy """

        if socket in self.stale_sockets.keys():
            k = self.stale_sockets[socket]
            del self.stale_sockets[socket]
            del self.serial_clients[k]
            with self.available_devices_lock:
                del self.available_devices[k]

if __name__ == "__main__":
    try:
        btpm = BluetoothPortManager(legacy_port=True)
        btpm.start()

    except KeyboardInterrupt:
        self.shutdown = True
        print("Keyboard Interrupt detected. Exiting...")
        exit(0)


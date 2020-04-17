#! /usr/bin/env python3.5

import bluetooth
from bluetooth.ble import DiscoveryService # bluetooth low energy scan

import threading
import time
import math
import subprocess
import yaml

# Logger levels
DEBUG = True
DEBUGG = True
DEBUGGG = False


class BluetoothPortManager:

    def __init__(self):
        self.poll_frequency = 1 #seconds
        self.scan_timeout = 3 #seconds
        self.address_timeout = 10 #seconds; time, after which an address is considered 'stale'
        self.clients = [] #who we're serving
        self.servers = [] #who we're connected to

        self.available_devices = dict() #BLE scan results and the last time they were seen
        self.available_devices_lock = threading.Lock()
        self.shutdown = False
        self.mac_whitelist = self._load_yaml("config/whitelist.yaml")
        self.my_mac = bluetooth.read_local_bdaddr()[0]
        self.my_nice = self.mac_whitelist[self.my_mac]['nice']

    def _sum_mac_addr(self, mac_addr):
        acc = 0
        for byte_pair in mac_addr.split(':'):
            acc += int(byte_pair[0], 16)
            acc += int(byte_pair[1], 16)
        return acc

    def _load_yaml(self, yaml_path):
        with open(yaml_path, 'r') as stream:
            return yaml.safe_load(stream)

    def run(self):
        # Setup address polling
        threading.Timer(self.poll_frequency, self.scan_addresses).start()

        try:
            while not self.shutdown:
                for addr in self.available_devices.keys():
                    if addr in self.mac_whitelist.keys():
                        if DEBUG:
                            print("Remote " + addr + " found in whitelist.")
                        remote_nice = self.mac_whitelist[addr]['nice']

                        # standard policy, greater nice value acts as server
                        if self.my_nice > remote_nice and addr not in self.clients:
                            #spin up server
                            if DEBUGG:
                                print("Found remote with smaller nice value. Hosting...")
                            self.clients.append(addr)
                            self.rfcomm_serve()

                        else:
                            if addr not in self.servers:
                                #spin up client
                                if DEBUGG:
                                    print("Found remote with greater nice value. Connecting...")
                                self.servers.append(addr)
                                self.rfcomm_client(addr)
                time.sleep(0.5)

        except KeyboardInterrupt:
            self.shutdown = True
            print("Keyboard Interrupt detected. Exiting...")

    def scan_addresses(self):
        le_devices = self.scan_addresses_low_energy()
        std_devices = self.scan_addresses_std()
        devices = le_devices + std_devices
        addrs = self.available_devices

        for address in devices:
            addrs[address] = math.ceil(time.time())

        #remove stale addresses
        for k in list(addrs.keys()):
            if time.time() - addrs[k] > self.address_timeout:
                del addrs[k]

        with self.available_devices_lock:
            self.available_devices = addrs

        if DEBUGGG:
            print("Available Devices: " + self.available_devices)

        if not self.shutdown:
            threading.Timer(self.poll_frequency, self.scan_addresses).start()


    def scan_addresses_low_energy(self):
        service = DiscoveryService()
        devices = []
        for addr, name in service.discover(self.scan_timeout).items():
            devices.append(addr)
        return devices
    """
    This scans for 'classic' bluetooth devices like those on Desktops
    This scan generally requires more time to pick up devices.
    """
    #TODO spin this as its own Timer
    def scan_addresses_std(self):
        return bluetooth.discover_devices(lookup_names=False, duration=self.scan_timeout)

    def bt_add_channel(self, channel_no):
        command = ['sdptool', 'add', '--channel=22', 'SP']
        result = subprocess.run(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        if result.returncode != 0:
            print("Unable to add channel via sdptool.")
            #TODO throw error
            pass

    def rfcomm_serve(self):
        self.bt_add_channel(22)
        command = ['rfcomm', 'watch', '/dev/rfcomm0', '22']
        result = subprocess.run(command, stdout=subprocess.DEVNULL)

    def rfcomm_client(self, server_mac):
        self.bt_add_channel(22)
        command = ['rfcomm', 'connect', '/dev/rfcomm0', str(server_mac), '22']
        result = subprocess.run(command, stdout=subprocess.DEVNULL)
        self.servers.remove(server_mac)

    def bt_port_server(self):

        server_sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )

        port = 1
        server_sock.bind(("",port))
        server_sock.listen(1)

        client_sock,address = server_sock.accept()
        #print("Accepted connection from ",address)

        data = client_sock.recv(1024)
        #print "received [%s]" % data

        client_sock.close()
        server_sock.close()

    def bt_port_client(self):
        bd_addr = "01:23:45:67:89:AB"

        port = 1

        sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )
        sock.connect((bd_addr, port))

        sock.close()

if __name__ == "__main__":
    btpm = BluetoothPortManager()
    btpm.run()


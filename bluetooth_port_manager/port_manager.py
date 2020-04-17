#! /usr/bin/env python3.5

import bluetooth
from bluetooth.ble import DiscoveryService # bluetooth low energy scan

import threading
import time
import math
import subprocess


class BluetoothPortManager:

    # Logger levels
    DEBUG = True
    DEBUGG = False
    DEBUGGG = False

    def __init__(self):
        self.poll_frequency = 1 #seconds
        self.scan_timeout = 2 #seconds
        self.address_timeout = 10 #seconds; time, after which an address is considered 'stale'
        self.mac_whitelist = []
        self.available_devices = dict() #BLE scan results and the last time they were seen
        self.available_devices_lock = threading.Lock()
        self.shutdown = False



    def run(self):
        # Setup address polling
        threading.Timer(self.poll_frequency, self.scan_addresses_low_energy).start()

        try:
            while not self.shutdown:
                time.sleep(0.5)

        except KeyboardInterrupt:
            self.shutdown = True
            print("Keyboard Interrupt detected. Exiting...")

    def scan_addresses_low_energy(self):
        service = DiscoveryService()
        devices = service.discover(self.scan_timeout)
        addrs = self.available_devices

        for address, name in devices.items():
            addrs[address] = math.ceil(time.time())
            if self.DEBUGG:
                print("address: {}".format(address))

        #remove stale addresses
        for k in list(addrs.keys()):
            if time.time() - addrs[k] > self.address_timeout:
                del addrs[k]

        with self.available_devices_lock:
            self.available_devices = addrs

        if self.DEBUG:
            print(self.available_devices)

        if not self.shutdown:
            threading.Timer(self.poll_frequency, self.scan_addresses_low_energy).start()

    """
    This scans for 'classic' bluetooth devices. Currently for reference only
    """
    def scan_addresses(self):
        names = False
        if names:
            nearby_devices = bluetooth.discover_devices(lookup_names=True, duration=3)
            print("Found {} devices.".format(len(nearby_devices)))
            for addr, name in nearby_devices:
                print("  {} - {}".format(addr, name))
        else:
            nearby_devices = bluetooth.discover_devices(lookup_names=False, duration=5)
            print("Found {} devices.".format(len(nearby_devices)))

            for addr in nearby_devices:
                print("  {}".format(addr))

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

        time.sleep(100)

    def rfcomm_client(self, server_mac):
        self.bt_add_channel(22)
        command = ['rfcomm', 'connect', '/dev/rfcomm0', str(server_mac), '22']
        result = subprocess.run(command, stdout=subprocess.DEVNULL)

        time.sleep(100)


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
    btpm.rfcomm_client("0C:DD:24:C0:AC:7D")
    #btpm.run()


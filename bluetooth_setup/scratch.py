#! /usr/bin/env python3

import bluetooth

if True:
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

	
# bluetooth low energy scan
from bluetooth.ble import DiscoveryService

service = DiscoveryService()
devices = service.discover(2)

for address, name in devices.items():
    print("name: {}, address: {}".format(name, address))

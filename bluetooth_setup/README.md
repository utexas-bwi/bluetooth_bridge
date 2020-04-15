


## INSTALLING AND SETTING UP BLUETOOTH COMMUNICATION VIA SERIAL PORTS

Required for bluetooth development:


sudo apt-get install libbluetooth-dev bluez bluez-tools

sudo pip3 install pygatt
sudo pip3 install pygattlib
sudo pip3 install pybluez

maybe:
USER must a member of the following groups:
	lp,netdev


SETTING UP SERIAL COMMUNICATION VIA BLUETOOTH

In this setup there was a server (Hermes, robot, 16.04) and a client (Kif, labcomputer, 16.04) 
Note: Many things were attempted before a solution was found, so some steps may be missing by nature of having them completed throughout the overall process. Fill these in as this guide is carried out on additional machines.



It's my understanding that these steps are required, initially, to start Bluez in compatibility mode:

$ sudo vim /etc/systemd/system/dbus-org.bluez.service

change the 

	ExecStart 

line to 
	ExecStart=/usr/lib/bluetooth/bluetoothd --compat

NOTE: systemctl edit dbus-org.bluez.service should be used to create an override file that won't be lost on package updates.

then run:
	$ systemctl daemon-reload
	$ systemctl restart bluetooth

this may have the effect of disabling bluetooth. Reenable it through the gnome applet (or figure out how to do it over CLI)

finally 
	$ chmod 777 /var/run/sdp


Hooray! Now we need to pair the bluetooth clients. This only needs to be done on one PC, as I believe pairing to be symmetric.

	$ bluetoothctl 
	# power on
	# agent on
	# default agent
	# scan on
	# pair <ADDRESS>
	# Ctrl+D

Okay. Now for the server, run:

	$  sdptool add --channel=22 SP

which adds a channel to the Bluetooth services module. Here we reserve channel 22 for use as a serial port (SP).
Now you can run either:
	$ sudo rfcomm listen /dev/rfcomm0 22
or 
	$ sudo rfcomm watch /dev/rfcomm0 22

the difference being listen will close when the client hangs up, and listen will spawn a new child process to continue listening for connections.

On the client side, you can run:

	$ sdptool add --channel=22 SP
	$ sudo rfcomm connect /dev/rfcomm0 <SERVER-BT-ADDRESS> 22

a successful connection is indicated by no errors, and the message "Connected /dev/rfcomm0 to <SERVER-BT-ADDRESS> on channel 22" on the client side, and "Connection from <CLIENT-BT-ADDRESS> to /dev/rfcomm0" on the Server Side


To test the serial connection, you can run a server side terminal with the command:
	
	$ cat /dev/rfcomm0

and on the client side, you can send serial data with screen with:

	$ sudo screen /dev/rfcomm0

which you should see the input to the screen as the output to the Server-side terminal.

If all is well, congrats! 



## SETTING UP ROS FOR SERIAL-TO-SERIAL COMMUNICATION



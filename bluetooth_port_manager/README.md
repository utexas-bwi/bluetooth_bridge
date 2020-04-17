In order to set permissions to give python access to system level bluetooth control, set permissions with: 

 $ sudo setcap 'cap_net_raw,cap_net_admin+eip' /usr/bin/python3.5

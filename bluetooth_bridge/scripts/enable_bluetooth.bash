#!/usr/bin/env bash
rfkill unblock bluetooth
sudo hciconfig hci0 up
sudo hciconfig hci0 piscan
#systemctl enable bluetooth.service
#service bluetooth start

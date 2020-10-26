#!/bin/bash

echo "remap the device serial port(ttyUSBX) to rosecho"
echo "check it using the command : ls -l /dev|grep ttyUSB"

sudo rm /etc/udev/rules.d/99-tianbot-rosecho.rules
sudo cp ./_udev_/99-tianbot-rosecho.rules  /etc/udev/rules.d

echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart


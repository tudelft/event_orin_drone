#!/usr/bin/env bash

# allow all usb for plugdev group
sudo cp 100-usb.rules /etc/udev/rules.d

# add user to video and plugdev groups
sudo usermod -aG video $USER
sudo usermod -aG plugdev $USER

# reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo service udev restart

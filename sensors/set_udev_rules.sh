#!/usr/bin/env bash

# https://gitlab.com/inivation/dv/libcaer/-/blob/master/docs/65-inivation.rules
sudo cp 65-inivation.rules /etc/udev/rules.d
echo "Udev rules have been written to /etc/udev/rules.d/65-inivation.rules and reloaded."

# https://github.com/IntelRealSense/librealsense/blob/master/config/99-realsense-libusb.rules
sudo cp 99-realsense-libusb.rules /etc/udev/rules.d
echo "Udev rules have been written to /etc/udev/rules.d/99-realsense-libusb.rules and reloaded."

# add user to video and plugdev groups
sudo usermod -aG video $USER
sudo usermod -aG plugdev $USER

# reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo service udev restart

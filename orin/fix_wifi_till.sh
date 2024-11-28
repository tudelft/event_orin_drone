#!/usr/bin/env bash

## from https://github.com/tudelft/dronerace/blob/master/orin/configure_orin.sh
## Download & install wifi drivers
wget https://cdn.kernel.org/pub/linux/kernel/projects/backports/stable/v5.15.153/backports-5.15.153-1.tar.xz
tar Jxfv  backports-5.15.153-1.tar.xz 
cd backports-5.15.153-1
make defconfig-iwlwifi
make -j8
sudo make install
sudo echo "options iwlwifi 11n_disable=1" >> /etc/modprobe.d/iwlwifi.conf

## solve ping issue by disable power save mode
sudo sed -i -e 's/wifi.powersave = 3/wifi.powersave = 2/g' /etc/NetworkManager/conf.d/default-wifi-powersave-on.conf

## disable bluetooth 
sudo sed -i -e 's/AutoEnable=true/AutoEnable=false/g' /etc/bluetooth/main.conf

## don't set fan to max

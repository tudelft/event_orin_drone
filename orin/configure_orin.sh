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

## set fan to max on boot
sudo echo "#!/bin/bash
jetson_clocks --fan" >>/usr/local/bin/set_fan_to_max.sh
sudo chmod +x /usr/local/bin/set_fan_to_max.sh
sudo echo "[Unit]
After=network.target

[Service]
ExecStart=/usr/local/bin/set_fan_to_max.sh

[Install]
WantedBy=default.target">>/lib/systemd/system/set_fan_to_max.service
sudo systemctl daemon-reload
sudo systemctl enable set_fan_to_max.service
sudo nvpmodel -m 3

echo "!!!!!! Make sure to shutdown and then start after this, restart is not the same as shutdown apparently !!!!"

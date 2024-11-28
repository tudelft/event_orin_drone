# Event-Orin drone

Overview repository for the Event-Orin drone.

```bash
git clone --recurse-submodules git@github.com:tudelft/event_orin_drone.git
```

## Repository structure

Airborne stuff:
- [fc](#fc): flight controller and software (PX4)
- [drone](#drone): drone parts and setup
- [orin](#orin): Jetson Orin setup and config
- [sensors](#sensors): sensor setup and config

Ground stuff:
- [gs](#gs): ground station software (QGroundControl)
- [sim](#sim): extras for PX4 simulation

Communication stuff:
- [comms](#comms): communication setup (MAVLink Router, uXRCE agent)
- [wifi](#wifi): wifi setup
- [radio](#radio): radio transmitter setup

Misc:
- [scripts](#scripts): some useful scripts

If a folder/link doesn't exist, there's nothing there (yet).

Tested on:
- Ubuntu 22.04 (WSL) on Windows 11 with NVIDIA GPU
- Jetson Orin NX 16 GB with JetPack 6.0 and Jetson Linux 36.3

### FC

- Holybro Kakute H7 Mini flight controller
- Disable ESC beeping/add a song:
    - [BLHeli Suite](https://github.com/bitdump/BLHeli/releases) (.zip file for windows)
    - Follow [this](https://oscarliang.com/connect-flash-blheli-32-esc/) guide roughly
        - No more flashing: [eol](https://oscarliang.com/end-of-blheli_32/#The-Issue-with-Firmware-Updates)
    - Add music: follow [this](https://oscarliang.com/blheli-32-custom-startup-tone-music/)
        - [Imperial march](https://www.youtube.com/watch?v=bsjB0JLju-0)
    - Disable inactivity beep: turn beacon strenth to zero as [here](https://drones.stackexchange.com/questions/2441/turn-off-beeping-of-esc-if-idle-for-some-time#:~:text=The%20inactivity%20beep%20is%20a,until%20it%20says%20%22Off%22)
- Flash Kakute H7 Mini flight controller with PX4 firmware
    - Initially has Betaflight, so use [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator/releases) to flash the bootloader following [this](https://docs.px4.io/main/en/advanced_config/bootloader_update_from_betaflight.html)
        - Doesn't always work?
        - No DFU mode, no expert mode, [.hex bootloader file](fc/holybro_kakuteh7mini_bootloader.hex)
    - Build firmware: `make holybro_kakuteh7mini_default` (file [here](fc/holybro_kakuteh7mini_default.px4))
    - Flash with QGroundControl
    - Upload [parameter file](fc/event-orin.params)

### Drone

**TODO: add full parts list**

### Orin

#### General

**TODO: add dts/dtb files**

- Jetson Orin NX 16GB module with [DAMIAO v1.1 carrier board](https://www.ebay.nl/sch/i.html?_nkw=DAMIAO+ORIN+NX+Carrier+Board&_sacat=0)
    - Careful installing the heat sink!
- Other hardware:
    - SSD: Corsair MP600 Core Mini
    - Wifi: Intel AX200 (add some antennas)
    - RTC battery: CR1220
    - Good XT30 power supply (used the one from A603 carrier board)
- Use [SDK manager](https://developer.nvidia.com/sdk-manager) to install JetPack 6.0
    - If not seen, make sure that it is in recovery mode (`lsusb` should show `NVIDIA Corp. APX`)
    - Select everything for installation
    - Select Jetson Orin NX 16GB with developer kit (even though it isn't)
    - **mavlab/mavlab** for login
    - If it doesn't work: try different cable! I had to retry a few times
        - Too hot? USB timeout? Cable?
- Update everything in GUI and with `apt update` (but don't yet `autoremove`)
- `hostnamectl set-hostname event-orin-drone`
- Configure wifi using `nmtui` or connect over ethernet for now (make sure to set to 'share to local' on Ubuntu)
- Fix wifi:
    - Run [`fix_wifi_till.sh`](orin/fix_wifi_till.sh)
    - Better version of [this](https://forums.developer.nvidia.com/t/jetpack-6-wifi-does-not-work-with-intel-ac-9260/292424)?
- Enable extra UART following [these](https://forums.developer.nvidia.com/t/orin-nano-jetpack-6-0-uart1-issue/299825/8) [posts](https://forums.developer.nvidia.com/t/issue-with-enabling-uartb-serial-3110000-on-orin-nx-with-l4t-36-3/297052):
  - Decompile dtb to dts with `sudo dtc -I dtb -O dts A.dtb > A.dts`, modify, then recompile with `sudo dtc -I dts -O dtb A.dts > A.dtb`
  - Put in `/boot/dtb`, then specify the file in `/boot/extlinux/extlinux.conf` as in the posts, then reboot
- Install `jtop` ([instructions](https://github.com/rbonghi/jetson_stats))
- Check RTC: [here](https://ubuntu.com/core/docs/system-time)
- UARTs:
    - FC `telem1` (connector on FC) -> `/dev/ttyTHS1`: UART closest to carrier board edge
        - MAVLink Router
    - FC `telem2` (soldered on FC) -> `/dev/ttyTHS3`: UART closest to center of carrier board
        - uXRCE agent to ROS

#### `jetson-containers`

- Install [jetson-containers](https://github.com/dusty-nv/jetson-containers/blob/master/docs/setup.md)
- Build base image with e.g. `jetson-containers build --name event-orin cuda:12.2 pytorch:2.4 openai-triton:3.0.0`
- Check versions of installed packages, so you can get pre-built binaries from [here](http://jetson.webredirect.org/jp6/cu122)
- When running a container, it's nice to create a user with the same UID/GID inside, and use that, so not all folders will end up being owned by root: `jetson-containers run -v .:/workspace --user $(id -u):$(id -g) <container_name>`
- Example script for drone flight: see [here](orin/drone_flight.sh)

### Sensors

#### General

- If you want to access sensors without `root` (whether inside or outside a container), we need to set some udev rules
- Use [sensors/unsafe_usb_udev.sh](sensors/unsafe_usb_udev.sh) (UNSAFE? but fine for Orin?) to open all USB ports for regular users
    - Test if this works
- On WSL, you need to share the USB device between Windows and WSL using [`usbipd-win`](https://github.com/dorssel/usbipd-win)
    - You can check if it's being detected in WSL with `lsusb`

#### MicoAir MTF-01

- Check that flow scale is set to 100 using [MicoAssistant](https://micoair.com/optical_range_sensor_mtf-01/)

#### Intel RealSense D435i

- Udev rules: [here](sensors/99-realsense-libusb.rules) from [here](https://github.com/IntelRealSense/librealsense/blob/master/config/99-realsense-libusb.rules)
- Build `librealsense` from source in Docker container following [here](https://github.com/dusty-nv/jetson-containers/blob/master/packages/hardware/realsense/Dockerfile)
    - Before, we had `realsense` in `jetson-containers`, but that still needed two fixes to allow `realsense` to build (from [here](https://github.com/dusty-nv/jetson-containers/issues/281#issuecomment-1874942046)).
    - Furthermore, `librealsense` libs should be loaded before any ROS2 ones, so we prepend to the `LD_LIBRARY_PATH` in the [`entrypoint.sh`](entrypoint.sh) script following [this comment](https://github.com/IntelRealSense/librealsense/issues/12831#issuecomment-2041140925).
- Run from Docker container using [`realsense-ros`](https://github.com/IntelRealSense/realsense-ros)
- Check if working with `rs-enumerate-devices`
- Could only get 15 Hz at 640x480 (even though 30 Hz should work?)

Configuration:
- Use [depth quality tool](https://github.com/IntelRealSense/librealsense/releases/tag/v2.56.2) to tune
    - Spatial, temporal and decimation filters with defaults
    - 640x40, 30 fps does work here, even with USB2.1
    - Both image and depth to auto-exposure
    - Align depth to color is nice
    - Further tips [here](https://dev.intelrealsense.com/docs/tuning-depth-cameras-for-best-performance#use-post-processing)

#### iniVation DVXplorer Micro

##### General

- Udev rules: [here](sensors/65-inivation.rules) from [here](https://gitlab.com/inivation/dv/libcaer/-/blob/master/docs/65-inivation.rules)
- Run from Docker container using [`libcaer_driver`](https://github.com/ros-event-camera/libcaer_driver) ROS packages
- Check if working with `ros2 launch libcaer_driver driver_node.launch.py`
- Works well, but seems to saturate for full resolution?

##### Camera calibration

- Could be done with a combination of [`simple_image_recon`](https://github.com/berndpfrommer/simple_image_recon) and [`camera_calibration`](https://docs.ros.org/en/rolling/p/camera_calibration/tutorial_mono.html), but it's easier to use [DV-GUI](https://docs.inivation.com/software/dv/gui/calibrate-event-camera.html) for this.
- Parameters that came out for our DVXplorer with 3.6mm lens can be found [here](sensors/dvx_calib_normal.xml)

### GS

Tested on Windows 11 with WSL (Ubuntu 22.04).

#### General

- [QGroundControl](https://github.com/mavlink/qgroundcontrol/releases) (stable, v4.4.2)
    - Install in Windows, else you can't flash
- Configure link to WSL: see [here](https://docs.px4.io/main/en/dev_setup/dev_env_windows_wsl.html)

#### Containers

- Install [Docker](https://docs.docker.com/engine/install/ubuntu/)
- Install [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- Install [jetson-containers](https://github.com/dusty-nv/jetson-containers/blob/master/docs/setup.md) (up to and including 'Docker default runtime')
    - On `aarch64`, `jetson-containers` automatically mounts USB devices, but this doesn't happen on [`x86_64`](https://github.com/dusty-nv/jetson-containers/blob/master/run.sh), so add `--device /dev/bus/usb`.

### Sim

If you want to run simulations with PX4 and Gazebo and ROS2:
- Install [PX4](fc/PX4-Autopilot/) following [this](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)
- ROS2 Humble following [this](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- ROS2-Gazebo bridge for Gazebo Garden following [this](https://gazebosim.org/docs/latest/ros_installation/#gazebo-garden-with-ros-2-humble-iron-or-rolling-use-with-caution)
- Micro XRCE-DDS Agent following [this](https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html#installation-from-snap-package) using `snap` with the `--edge` tag
    - Run with `snap run micro-xrce-dds-agent udp4 -p 8888`
- Connect radio as joystick, calibrate in Windows/QGroundControl
    - Joystick buttons: some issue [here](https://github.com/mavlink/qgroundcontrol/issues/4520)
- Simulation with depth camera messages sent to ROS2: [`sim/px4_gazebo_sim_depth.sh`](sim/px4_gazebo_sim_depth.sh)
- If you need it to work with a container, set `--ipc=host` following [this](https://github.com/eProsima/Fast-DDS/issues/2956)

### Comms

#### MAVLink Router

- For telemetry to ground station and logging (because PX4 cannot log to flash, and Kakute H7 Mini doesn't have SD...)
- Install [mavlink-router](https://github.com/mavlink-router/mavlink-router)
    - Make sure to add user to `dialout` and `tty` as [here](https://unix.stackexchange.com/questions/14354/read-write-to-a-serial-port-without-root)
- Put [config](comms/mavlink-router.conf) (from [here](https://bellergy.com/6-install-and-setup-mavlink-router/)) in `/etc/mavlink-router/main.conf`
- Run as `mavlink-routerd`
- Make sure to set up a TCP comms link in QGroundControl pointing to `orin-ip:5760`

#### uXRCE agent

- For communication between PX4 and ROS2
- Install following [this](https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html#installation-from-snap-package) using `snap`
- Run as `micro-xrce-dds-agent serial --dev /dev/ttyTHS3 -b 921600`

#### PX4-ROS2

- PX4-ROS2 interface lib as [here](https://docs.px4.io/main/en/ros2/px4_ros2_interface_lib.html)
- Works great
- Custom modes don't show up in QGroundControl v4.4.2 but you can still select them by overwriting PX4 default modes
    - In PX4 shell: `commander status` to see if registered, then `commander takeoff` and `commander mode auto:mission` to switch to the custom mode overwriting the PX4 'Mission' flight mode

### Wifi

- Tailscale
    - Fixed some problems: [this](https://tailscale.com/kb/1188/linux-dns)
    - Unstable? Sometimes doesn't work and need reboot
    - eduroam seems too high latency?
- Key-only SSH, no root login
- Hotspot on laptop or phone

**TODO: how to get W11 hotspot working with WSL and Orin?**
- Maybe setup hotspot on Orin? `event-orin-drone`/`mavlabcool`

### Radio

- ELRS, Radiomaster Pocket with EdgeTX, Radiomaster RP2
- Setup:
    - Update EdgeTx with [buddy](https://manual.edgetx.org/installing-and-updating-edgetx/update-from-opentx-to-edgetx-1), and doing reinstall in SD card content to get clean install
    - Go through all radio settings with [these](https://oscarliang.com/setup-radiomaster-pocket/) [posts](https://manual.edgetx.org/bw-radios)
    - Then continue here https://www.expresslrs.org/quick-start/transmitters/tx-prep/#radio-settings
    - Then update ELRS Tx firmware [here](https://www.expresslrs.org/quick-start/transmitters/updating/#via-etx-passthrough)
        - Better instructions [here](https://oscarliang.com/update-zorro-elrs-firmware/)
	- Configurator [here](https://www.expresslrs.org/quick-start/installing-configurator/)
	- **Bind phrase: `mavlab is cool`**
	- Receiver: follow [this](https://www.expresslrs.org/quick-start/receivers/updating/#manual-upload-via-ap)
    - Packet [rate](https://www.expresslrs.org/quick-start/transmitters/lua-howto/?h=packet#packet-rate-and-telemetry-ratio)
- Don't use MAVLink telemetry, just use CRSF protocol (link mode normal), packet rate 250Hz I think, switch mode wide, telem ratio std
    - Also check receiver via 'other devices' (?)
- Disable auto wifi interval: see [this](https://github.com/ExpressLRS/ExpressLRS/issues/1797) (annoying)

### Scripts

- [`move_logs_to_drive.sh`](scripts/move_logs_to_drive.sh): mount USB stick, move logs, unmount

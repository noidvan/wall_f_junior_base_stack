# WALL-F Junior Base Stack

## Raspberry Pi & Base Station Software Installation

### Install Ubuntu

Install **Ubuntu Desktop 24.02.2 LTS (64-bit)** to Raspberry Pi 4 boot SD card using [Raspberry Pi Imager](https://www.raspberrypi.com/software/)

```bash
sudo apt update
sudo apt upgrade
sudo apt install vim
```

### Enable SSH (Raspberry Pi Only)
```bash
sudo apt install ssh
sudo systemctl enable ssh
```

### Install ROS

Install **ROS 2 Jazzy** following official [installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
sudo rosdep init
rosdep update
```

### Adjust CSI Camera Permissions (Raspberry Pi Only)
```bash
sudo vim /etc/udev/rules.d/raspberrypi.rules
# SUBSYSTEM=="dma_heap", GROUP="video", MODE="0660"
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG video $USER
```

### Enable Additional UART (Raspberry Pi Only)
```bash
sudo usermod -a -G tty $USER
sudo vim /boot/config.txt
```

Add the following lines and reboot:
```
enable_uart=1
dtoverlay=uart0
dtoverlay=uart1
dtoverlay=uart2
dtoverlay=uart3
dtoverlay=uart4
dtoverlay=uart5
```

### Install Dependencies of Raspberry Pi Pico C/C++ SDK
```bash
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
```

### Build
```bash
git clone --recurse-submodules https://github.com/noidvan/wall_f_junior_base_stack
cd wall_f_junior_base_stack
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Credits

This project contains code from the following open-source releases:
* [ROS 2 node for libcamera](https://github.com/christianrauch/camera_ros) by Christian Rauch under the MIT license
* [micro-ROS Agent](https://github.com/micro-ROS/micro-ROS-Agent) by Open Robotics under the Apache-2.0 license
* [Autonomy Stack](https://github.com/jizhang-cmu/autonomy_stack_mecanum_wheel_platform) by Prof. Ji Zhang
* [micor-ROS Raspberry Pi Pico SDK](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk) by Open Robotics under the Apache-2.0 license
* [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) by Raspberry Pi under the BSD-3-Clause license
* [Pico BME688](https://github.com/bablokb/pico-bme688) by bablokb
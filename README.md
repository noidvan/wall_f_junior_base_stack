# WALL-F Junior Base Stack

## Raspberry Pi Software Installation

### Install Ubuntu

Install **Ubuntu Desktop 24.02.2 LTS (64-bit)** to Raspberry Pi 4 boot SD card using [Raspberry Pi Imager](https://www.raspberrypi.com/software/)

```bash
sudo apt update
sudo apt upgrade
sudo apt install vim
```

### Enable SSH
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

### Adjust CSI Camera Permissions
```bash
sudo vim /etc/udev/rules.d/raspberrypi.rules
# SUBSYSTEM=="dma_heap", GROUP="video", MODE="0660"
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG video $USER
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
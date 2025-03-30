# WALL-F Junior Base Stack

## Start System
SSH into RPI terminal and execute: `./system_rpi.sh`

On base station terminal execute: `./system_base_station.sh`

## Hardware Connection
### Rear Driver Motor Driver (L298N)
- 12V -> Battery pack + 
- GND -> Battery pack - & Pico rear GND 
- ENA -> Pico rear GP6 
- IN1-> Pico rear GP7 
- IN2-> Pico rear GP8 
- IN3-> Pico rear GP9 
- IN4-> Pico rear GP10 
- ENB-> Pico rear GP11 

### Soil Moisture Sensor Stepper Motor Driver (A4988)
- VM-> Battery pack + 
- GND (bellow VM) -> Battery pack - 
- 1B, 1A, 2A, 2B -> Servo motor 4 pin plug, order doesn’t matter 
- VDD -> Pico rear 3V3(OUT) 
- GND (bellow VDD) -> Pico rear GND 
- MS2 -> Pico rear 3V3(OUT) 
- RST & SLP -> Jumped together with jumpers 
- STEP -> Pico rear GP12 
- DIR-> Pico rear GP13 

### Soil Moisture Sensor
- Red wire -> Pico rear ADC_VREF 
- Black wire -> Pico rear AGND 
- The one that is not black nor red -> Pico rear GP26 

### Air Sensor (BME680)
- 2-5V -> Pico rear 3V3(OUT) 
- SDA -> Pico rear GP2 
- SCL -> Pico rear GP3 
- GND -> Pico rear GND 

### Left Joint Stepper Motor Driver (A4988)
- VM-> Battery pack + 
- GND (bellow VM) -> Battery pack -  
- 1B, 1A, 2A, 2B -> Servo motor 4 pin plug
- VDD -> Pico front 3V3(OUT) 
- GND (bellow VDD) -> Pico front GND 
- MS2 -> Pico front 3V3(OUT) 
- RST & SLP -> Jumped together with jumpers 
- STEP -> Pico front GP6 
- DIR-> Pico front GP7 

### Right Joint Stepper Motor Driver (A4988)
- VM-> Battery pack + 
- GND (bellow VM) -> Battery pack -  
- 1B, 1A, 2A, 2B -> Servo motor 4 pin plug
- VDD -> Pico front 3V3(OUT) 
- GND (bellow VDD) -> Pico front GND 
- MS2 -> Pico front 3V3(OUT) 
- RST & SLP -> Jumped together with jumpers 
- STEP -> Pico front GP8
- DIR-> Pico front GP9

### IMU (GY-521)
- VCC -> Pico front 3V3(OUT) 
- GND -> Pico front GND 
- SCL -> Pico front GP3 
- SDA -> Pico front GP2 

### First Section Driver Motor Driver (L298N)
- 12V -> Battery pack + 
- GND -> Battery pack - & Pico first GND 
- ENA -> Pico first GP6 
- IN1-> Pico first GP7 
- IN2-> Pico first GP8 
- IN3-> Pico first GP9 
- IN4-> Pico first GP10 
- ENB-> Pico first GP11 

### 5V Regulator
- IN+ -> Battery pack + 
- IN- -> Battery pack - 
- OUT+ -> Pico first VSYS, Pico rear VSYS & Pico front VSYS 
- OUT- -> Pico first GND, Pico rear GND & Pico front GND 

### Raspberry Pi
- GND -> Pico rear GND & Pico front GND 
- GPIO4 -> Pico first GP1 
- GPIO5 -> Pico first GP0 
- GPIO8 -> Pico rear GP1 
- GPIO9 -> Pico rear GP0 
- GPIO12 -> Pico front GP1 
- GPIO13 -> Pico front GP0  

## Software Installation

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
sudo usermod -a -G video $USER
```

### Enable Additional UART (Raspberry Pi Only)
```bash
sudo usermod -a -G dialout $USER
sudo vim /boot/firmware/config.txt
```

Add the following lines and reboot:
```
enable_uart=1
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

### Flash Pico
* After compilation, locate `front_agent.uf2`,  `rear_agent.uf2` and `first_section_agent.uf2` in `build/wall_f_junior_pico`.
* Press and hold the `BOOTSEL` button on the Pico while plugging in its USB to the computer.
* Copy the `.uf2` file for the corresponding Pico into the appeared drive

## Credits

This project contains code from the following open-source releases:
* [ROS 2 node for libcamera](https://github.com/christianrauch/camera_ros) by Christian Rauch under the MIT license
* [micro-ROS Agent](https://github.com/micro-ROS/micro-ROS-Agent) by Open Robotics under the Apache-2.0 license
* [Autonomy Stack](https://github.com/jizhang-cmu/autonomy_stack_mecanum_wheel_platform) by Prof. Ji Zhang
* [micor-ROS Raspberry Pi Pico SDK](https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk) by Open Robotics under the Apache-2.0 license
* [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) by Raspberry Pi under the BSD-3-Clause license
* [Pico BME688](https://github.com/bablokb/pico-bme688) by bablokb

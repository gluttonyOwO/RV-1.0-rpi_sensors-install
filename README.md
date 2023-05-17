# rpi_sensors

*`Updated: 2023/05/17`*

Web version: https://hackmd.io/@cocobird231/BJidBuC6i

:::    warning
**Guidance for hardware installation:** [rpi_sensors Hardware Installation](https://hackmd.io/@cocobird231/rkUS37lKc)
:::

The sensor installation pack for Raspberry Pi 4. The installation will automatically detect and install dependencies. The ROS2 environment is based on Docker.

**System requirements**:
- OS: Raspberry Pi OS (64-bit recommend)
- RAM: 4G or higher

:::info
[How to write Raspberry Pi OS into SD card?](https://hackmd.io/@cocobird231/S1eCXHR6i)
:::

**Now supported sensor types**:
- GPS module (MAX-M8Q GNSS, ZED-F9P HAT)
- SenseHat module (IMU and environment sensors)
- RF Communication module (send and receive)
- Ultrasound module (HC-SR04 sensors)
- Webcam module (Based on OpenCV4)

:::warning
The following image instructions use webcam module installation as an example.
:::

---
## Usage

### For Newly Install (No `ros2_docker` Under Home Path)
Run the script`get_rpi_sensors.sh` to grab git controlled SensorPack directory (renamed as ros2_docker). **Make sure Raspberry Pi 4 is connected to the internet before installation.**

1. Run the script `get_rpi_sensors.sh`. The new directory `ros2_docker` will be created under `$HOME`.
    ```bash
    $. get_rpi_sensors.sh
    ```

### Module Program Installation

1. Run the script `install.sh` under `ros2_docker` to install program for selected module.
    ```bash
    $. install.sh
    
    # Select a number for module installation
    ```
2. Determine the network interface and IP for program to executing.
3. Reboot while installation finished. The program will be running at startup.

:::warning
**Effects After Installation**
- Files Creation:
    - Under `$HOME/ros2_docker`
        - run.sh.tmp
        - Dockerfile.tmp
        - requirement_apt.txt (link to selected module pack)
        - requirement_pip.txt (link to selected module pack)
        - source_env.txt (link to selected module pack)
        - .modulename (selected module pack name)
        - .moduleinterface (interface setting)
        - .modulename (IP setting)
    - Under System Environment
        - /boot/config.txt.tmp
        - /etc/dhcpcd.conf.tmp
        - /etc/xdg/autostart/ros2_docker.desktop (startup)
- Dependencies Installation
    - python3 python3-dev python3-pip git curl
    - Docker
    - APT installed packages were listed under requirement_apt.txt.
    - Python3 pip installed packages were listed under requirement_pip.txt.
:::


### Module Program Code Update
1. Run the script `install.sh` under `ros2_docker` directory and enter `u` for update process.
    ```bash
    $. install.sh
    
    # Enter 'u' for update process
    ```
2. The update process will update `codePack` under `ros2_docker` by pulling repositories from git server.
3. After pulling, the module program will start rebuilding if module program had been installed before.

---
### Module Parameters Setting for ROS2
Settings may be varient in different sensors, but there are some common parameters need to be changed:
1. Device node name (#primary tag)
2. Topic name (may be one or more)
3. Publish interval (Need to be float, e.g. not `1` but `1.0`)

Modify the settings under `~/ros2_docker/common.yaml` and reboot device.
![](https://i.imgur.com/BBYVW08.png)

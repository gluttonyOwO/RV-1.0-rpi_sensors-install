# Raspberry Pi Sensors Installation

Web version: https://hackmd.io/@cocobird231/BJidBuC6i

*`Updated: 2024/04/26`*

:::warning
**Guidance for hardware installation:** [rpi_sensors Hardware Installation](https://hackmd.io/@cocobird231/rkUS37lKc)
:::

The package installer for sensor depends on Raspberry Pi 4. The ROS2 environment is based on Docker.

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
- WebRTC streaming camera (Based on GStreamer)

---
## Usage

### For Newly Install (No `ros2_docker` Under Home Path)
Run the pre-install script`get-rpi-sensors-install.sh` to grab git controlled SensorPack directory (renamed as `ros2_docker`). **Make sure Raspberry Pi 4 is connected to the internet before installation.**
```bash
curl -fsSL ftp://61.220.23.239/rv-12/get-rpi-sensors-install.sh | bash
```
The new directory `ros2_docker` will be created under `$HOME`.

### Module Installation

1. Run the script `install.sh` under `ros2_docker` to install package for selected module.
    ```bash
    . ~/ros2_docker/install.sh
    ```
    Select a number for module installation
2. Determine the network interface and IP for program to execute.
3. Reboot while installation finished. The program will be running at startup automatically.

:::warning
**Effects After Installation**
- Files Creation:
    - Under `$HOME/ros2_docker`
        - `run.sh.tmp`
        - `Dockerfile.tmp`
        - `.modulename` (selected module pack name)
        - `.moduleinterface` (interface setting)
        - `.modulename` (IP setting)
        - `common.yaml` (copy from codePack package)
        - `service.json` (copy from codePack package)
    - Under System Environment
        - `/boot/config.txt.tmp`
        - `/etc/dhcpcd.conf.tmp`
        - `/etc/xdg/autostart/ros2_docker.desktop` (startup)
- Dependencies Installation
    - `python3` `python3-dev` `python3-pip` `git` `curl`
    - `Docker`
    - APT installed packages were listed under `requirement_apt.txt`.
    - Python3 pip installed packages were listed under `requirement_pip.txt`.
:::


### Module Update
1. Run the script `install.sh` under `ros2_docker` directory and enter `u` for update process.
    ```bash
    . install.sh
    
    # Enter 'u' for update process
    ```
2. The update process will update `codePack` under `ros2_docker` by pulling repositories from git server.
3. After pulling, the module program will start rebuilding if module program had been installed before.

### Module Removal
1. Run the script `install.sh` under `ros2_docker` directory and enter `r` for package remove process.
    ```bash
    . install.sh
    
    # Enter 'r' for update process
    ```
The files which describes at **Effects After Installation** section will be removed except the files installed from `requirement_apt.txt` and `requirement_pip.txt`.

- **Remove docker image (option)**
    check container
    ```bash
    sudo docker ps -a
    ```
    stop and delete container if running
    ```bash
    sudo docker stop <container_id>
    ```
    ```bash
    sudo docker rm <container_id>
    ```
    check images
    ```bash
    sudo docker images
    ```
    delete image
    ```bash
    sudo docker rmi <image_id>
    ```

### Parameters Setting for ROS2
Settings may be varient in different sensors, but there are some common parameters need to be changed:
1. Device node name (under `generic_prop` tag)
2. Device ID (under `generic_prop` tag)
3. Topic name (may be one or more)
4. Publish interval (Need to be float, e.g. not `1` but `1.0`)

Modify the settings under `$HOME/ros2_docker/common.yaml` and reboot device.
![](https://hackmd.io/_uploads/ByKJA8Syp.png)

---

### One Line Command
:::success
The `install.sh` now supported parser installation.
:::
The package installation, updating and removal functions can be done by adding some arguments while running `install.sh`.

#### **Overall arguments**
```bash!
. install.sh [[-i|--install] <package_name>] [--interface <network_interface>] [--ip [<static_ip>|dhcp]] [-rm|--remove] [-u|--update] [-p|--preserve]
```
- Command explanation
    - **`-i|--install`**: install specific package from codePack to Docker.
    - **`-rm|--remove`**: remove package settings and environment settings.
    - **`-u|--update`**: update codePack without install packages.
    - **`-p|--preserve`**: preserve `common.yaml` file during installation.
    - **`--interface`**: determine the network interface during installation.
    - **`--ip`**: determine the network ip during installation.
- Variable explanation
    - **package_name**: real package name under codePack. If variable set to `auto`, the process will automatically detect current installed module settings, then install the packages.
    - **network_interface**: the network interface to detect network or internet connection, e.g. `eth0` or `wlan0`.
    - **static_ip**: the format should be like `ip/mask`, e.g. `192.168.1.10/16`.

:::info
- **For Commands**

The three commands `-i`, `-u` and `-rm` can be work independently. The priority order of the three commands from high to low are: `-rm` > `-u` > `-i`. That is, if three commands exists at the same time, the process will be:
1. Remove installed package settings and environment settings. (The Docker container will be preserved)
2. Update codePack without install any packages.
3. Install packages from codePack to Docker.

The flag `-p` tells the installer to keep old `common.yaml` file. If `-p` set but `-i` not set, the `-p` will be ignored. If `-p` set but the `common.yaml` file not found, the preservation will be ignored.

The `--interface` determines the network interface for network detection or internet detection while installed program startup. The `--interface` will be ignored if `-i` not set. The `--interface` is not necessary and will be set to default `eth0`.

The `--ip` can be set to `dhcp` and `<static_ip>`. If `<static_ip>` used, The `/etc/dhcpcd.conf` will be configured. The `--ip` is not necessary and will be set to default `dhcp`.
:::

:::warning

- **For Variables**

The default value of the variables describes as following:
- `package_name`: `NONE`
- `network_interface`: `eth0`
- `static_ip`: `NONE`

The `package_name` is necessary if `-i` set. The valid name of `package_name` were shown under codePack. If the package had installed before, set `package_name` to `auto` is valid for process to auto detect the package name.

If `package_name` set to `auto`, the process will ignored `--interface` and `--ip` settings.

The `network_interface` is necessary if `--interface` set. The `network_interface` do not have any valid check mechanism, be careful of the mistyping.

The `static_ip` is necessary if `--ip` not set to `dhcp`.
:::

#### **Examples**
- Install package
    ```bash!
    . install.sh -i <package_name> [--interface <network_interface>] [--ip [<static_ip> | dhcp]]
    ```
- Update codePack
    ```bash!
    . install.sh -u
    ```
- Update codePack then install package
    ```bash!
    . install.sh -i <package_name> [--interface <network_interface>] [--ip [<static_ip> | dhcp]] -u
    ```
- Update current package while preserve `common.yaml`
    ```bash!
    . install.sh -u -i auto -p
    ```
- Remove current package
    ```bash
    . install.sh -rm
    ```
- Remove current package, update codePack and install package
    ```bash!
    . install.sh -rm -u -i <package_name> [--interface <network_interface>] [--ip [<static_ip> | dhcp]]
    ```
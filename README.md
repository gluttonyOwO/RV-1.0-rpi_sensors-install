# rpi_sensors

Web version: https://hackmd.io/@cocobird231/BJidBuC6i

The sensor installation pack for Raspberry Pi 4. The installation will detect and install Docker automatically. The ROS2 environment is based on Docker.

System requirements:
- OS: Raspberry Pi OS (64-bit recommend)
- RAM: 4G or higher

Automatically installed:
- Docker
- ROS2: Humble distro (official Dockerfile)

Now supported sensor types:
- GPS module (MAX-M8Q GNSS HAT)
- SenseHat module (IMU and environment sensors)
- RF Communication module (send and receive)
- Ultrasound module (Three HC-SR04 sensors)
- Webcam module (Based on OpenCV4)

:::warning
The following picture was demonstrated as webcam module installation.
:::

## Usage

### For New Raspberry Pi 4 Device (no rpi_sensors installed)
The following step is for the Raspberry Pi 4 that did not installed rpi_sensors before. **Make sure Raspberry Pi 4 is connected to the internet before installation.**

1. Copy `rpi_sensors` directory to your USB storage device
2. Plug USB storage device into Raspberry Pi 4 and open `Terminal` under `rpi_sensors` directory
3. Run `install.sh` under terminal: `. install.sh`
![](https://i.imgur.com/tFBSjGO.png)
4. Follow the guide to finish installation. The files will be installed at `ros2_docker` under `HOME` directory.
![](https://i.imgur.com/HhTV4m6.png)
![](https://i.imgur.com/jmmh5C7.png)

:::warning
While finishing the installation, there were three changes in Raspberry Pi 4:
1. New `ros2_docker` directory under `Home` directory
2. New `ros2_docker.desktop` file under `/etc/xdg/autostart/`
3. If interface sets to static-ip, the file `dhcpcd.conf` under `/etc` will be modified. Remember to delete the old settings inside `/etc/dhcpcd.conf` manually if interface setting changes.
:::


### For rpi_sensors Exists (update code for same sensor module)
The following step is for the Raspberry Pi 4 that has been installed rpi_sensors before.

1. Copy code directory under `rpi_sensors/codePack` to `~/ros2_docker/codePack` (e.g. `cp -r rpi_sensors/cpp_webcam ~/ros2_docker/codePack/`)
2. Open `Terminal` under `~/ros2_docker` and run `. install_docker.sh -i INTERFACE`, the `INTERFACE` can be `eth0` or `wlan0`
![](https://i.imgur.com/mDoPED2.png)
3. Modify the sensor settings under `~/ros2_docker/codePack/{PACKAGE}/launch/common.yaml` and reboot device.


### Sensor ROS2 Parameters Setting
Settings may be varient in different sensors, but there are some common parameters need to be changed:
1. Device node name (#primary tag)
2. Topic name (may be one or more)
3. Publish interval (Need to be float, e.g. not `1` but `1.0`)

Modify the sensor settings under `~/ros2_docker/codePack/{PACKAGE}/launch/common.yaml` and reboot device.
![](https://i.imgur.com/BBYVW08.png)

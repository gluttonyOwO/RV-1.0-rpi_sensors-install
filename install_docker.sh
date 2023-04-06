#!/usr/bin/bash
echo "Run install_docker.sh"
static_ip="NONE"
package_name="NONE"
while [[ $# -gt 0 ]]; do
  case $1 in
    -i|--interface)
      interface="$2"
      shift # past argument
      shift # past value
      ;;
    --ip)
      static_ip="$2"
      shift # past argument
      shift # past value
      ;;
    -p|--package)
      package_name="$2"
      shift # past argument
      shift # past value
      ;;
    -*|--*)
      echo "Unknown option $1"
      exit 0
      ;;
    *)
      shift # past argument
      ;;
  esac
done



if [ "$package_name" != "NONE" ]; then
    echo "Installing..."
else
    echo "No package select. Exiting..."
    exit 1
fi

sudo apt update
sudo apt install python3 python3-dev python3-pip git curl -y
if [ -x "$(command -v docker)" ]; then
    echo "Found docker." && docker -v
else
    echo "No docker. Installing docker..."
    curl -fsSL https://get.docker.com -o get-docker.sh
    sudo sh get-docker.sh
fi
cp Dockerfile Dockerfile.tmp
line_default=19
sed -i "${line_default}r requirement_apt.txt" Dockerfile
echo "RUN . /opt/ros/\${ROS_DISTRO}/setup.sh && colcon build --packages-select $package_name vehicle_interfaces" >> Dockerfile

sudo docker build -t ros2_docker .
rm -rf Dockerfile && mv Dockerfile.tmp Dockerfile
sudo chmod a+x run.sh

rm -rf ros2_docker.desktop.tmp && touch ros2_docker.desktop.tmp
echo "[Desktop Entry]" >> ros2_docker.desktop.tmp
echo "Name=ros2_docker" >> ros2_docker.desktop.tmp
echo "Exec=lxterminal -e bash -c '$HOME/ros2_docker/run.sh $interface;\$SHELL'" >> ros2_docker.desktop.tmp
echo "Terminal=true" >> ros2_docker.desktop.tmp
sudo cp ros2_docker.desktop.tmp /etc/xdg/autostart/ros2_docker.desktop
rm -rf ros2_docker.desktop.tmp

sudo sed -i "s/#hdmi_force_hotplug=1/hdmi_force_hotplug=1/1" /boot/config.txt
sudo sed -i "s/#hdmi_group=1/hdmi_group=1/1" /boot/config.txt
sudo sed -i "s/#hdmi_mode=1/hdmi_mode=16/1" /boot/config.txt
sudo sed -i "s/#hdmi_drive=2/hdmi_drive=2/1" /boot/config.txt

if [ "$static_ip" != "NONE" ]; then
    sudo echo "interface $interface" >> /etc/dhcpcd.conf
    sudo echo "static ip_address=$static_ip" >> /etc/dhcpcd.conf
else
    echo "dhcpcd.conf not changed"
fi





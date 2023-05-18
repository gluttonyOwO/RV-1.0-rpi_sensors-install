#!/usr/bin/bash
while [[ $# -gt 0 ]]; do
  case $1 in
    -e|--extension)
      EXTENSION="$2"
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

target_dir="$HOME/ros2_docker"

PreparePackage ()
{
    # Check pwd
    if [ "$PWD" == "$target_dir" ]
    then
        echo "In $target_dir"
    else
        if ls $target_dir &> /dev/null
        then
            cd $target_dir
            echo "Change directory: $PWD"
        else
            echo "ros2_docker path error. Please copy ros2_docker directory under $HOME"
            exit 1
        fi
    fi
    # pwd in ~/ros2_docker

    # Store selected module name into .modulename file
    touch .modulename
    echo $pack_name > .modulename

    # Link requirement file to ~/ros2_docker for Dockerfile installation
    rm -rf requirement_apt.txt && ln codePack/$pack_name/requirement_apt.txt requirement_apt.txt
    rm -rf requirement_pip.txt && ln codePack/$pack_name/requirement_pip.txt requirement_pip.txt
    rm -rf source_env.txt && ln codePack/$pack_name/source_env.txt source_env.txt

    # Link common.yaml file to ~/ros2_docker for convenient modifying
    rm -rf common.yaml && ln codePack/$pack_name/launch/common.yaml common.yaml

    # Recover run.sh if .tmp exist
    if cat run.sh.tmp &> /dev/null
    then
        cp run.sh.tmp run.sh
        echo "run.sh recovered"
    else
        cp run.sh run.sh.tmp
        echo "Backup run.sh: run.sh.tmp"
    fi
    
    # Modify run.sh by adding specific $pack_name source_env.txt and docker run process
    cat source_env.txt >> run.sh
    echo "sudo docker run -v ~/ros2_docker/codePack/$pack_name/launch/common.yaml:/ros2_ws/install/$pack_name/share/$pack_name/launch/common.yaml --rm --privileged --net host -it ros2_docker ros2 launch $pack_name launch.py" >> run.sh
    sudo chmod a+x run.sh

    # Network Interface Selection
    echo "Enter network interface (default eth0):"
    read interface
    if [ $interface ]
    then
        echo "Interface: $interface"
    else
        interface="eth0"
        echo "Default interface: $interface"
    fi

    # Network IP Selection
    echo "Use DHCP? (y/n):"
    read static_ip
    if [[ "$static_ip" == "y" || "$static_ip" == "Y" ]]
    then
        static_ip="NONE"
    else
        echo "Enter static ip (ex 192.168.3.100/16):"
        read static_ip
        if [ ! $static_ip ]
        then
            static_ip="NONE"
        fi
    fi
    echo "Static IP: $static_ip"

    # Stored selected interface and ip
    touch .moduleinterface
    echo $interface > .moduleinterface
    touch .moduleip
    echo $static_ip > .moduleip
}

InstallDockerfile ()
{
    # Install Dockerfile Process
    echo "Installing dockerfile..."

    # Required environment installation and update
    sudo apt update
    sudo apt install python3 python3-dev python3-pip git curl -y

    # Check Docker
    if [ -x "$(command -v docker)" ]; then
        echo "Found docker." && docker -v
    else
        echo "No docker. Installing docker..."
        curl -fsSL https://get.docker.com -o get-docker.sh
        sudo sh get-docker.sh
    fi

    # Recover Dockerfile if .tmp exist
    if cat Dockerfile.tmp &> /dev/null
    then
        cp Dockerfile.tmp Dockerfile
        echo "Dockerfile recovered"
    else
        cp Dockerfile Dockerfile.tmp
        echo "Backup Dockerfile: Dockerfile.tmp"
    fi

    # Modify Dockerfile by adding requirement list
    line_default=19
    sed -i "${line_default}r requirement_apt.txt" Dockerfile
    echo "RUN . /opt/ros/\${ROS_DISTRO}/setup.sh && colcon build --packages-select $pack_name vehicle_interfaces" >> Dockerfile
    
    # Dockerfile Installation
    sudo docker build -t ros2_docker .
}

EnvSetting ()
{
    # Create ros2_docker.desktop file
    rm -rf ros2_docker.desktop.tmp && touch ros2_docker.desktop.tmp
    echo "[Desktop Entry]" >> ros2_docker.desktop.tmp
    echo "Name=ros2_docker" >> ros2_docker.desktop.tmp
    echo "Exec=lxterminal -e bash -c '$HOME/ros2_docker/run.sh $interface;\$SHELL'" >> ros2_docker.desktop.tmp
    echo "Terminal=true" >> ros2_docker.desktop.tmp

    # Copy ros2_docker.desktop to autostart directory
    sudo cp ros2_docker.desktop.tmp /etc/xdg/autostart/ros2_docker.desktop
    rm -rf ros2_docker.desktop.tmp

    # Recover /boot/config.txt if .tmp exist
    if sudo cat /boot/config.txt.tmp &> /dev/null
    then
        sudo cp /boot/config.txt.tmp /boot/config.txt
        echo "/boot/config.txt recovered"
    else
        sudo cp /boot/config.txt /boot/config.txt.tmp
        echo "Backup /boot/config.txt: /boot/config.txt.tmp"
    fi

    # Modify /boot/config.txt for booting behavior
    sudo sed -i "s/#hdmi_force_hotplug=1/hdmi_force_hotplug=1/1" /boot/config.txt
    sudo sed -i "s/#hdmi_group=1/hdmi_group=1/1" /boot/config.txt
    sudo sed -i "s/#hdmi_mode=1/hdmi_mode=16/1" /boot/config.txt
    sudo sed -i "s/#hdmi_drive=2/hdmi_drive=2/1" /boot/config.txt

    # Recover /etc/dhcpcd.conf if .tmp exist
    if sudo cat /etc/dhcpcd.conf.tmp &> /dev/null
    then
        sudo cp /etc/dhcpcd.conf.tmp /etc/dhcpcd.conf
        echo "/etc/dhcpcd.conf recovered"
    else
        sudo cp /etc/dhcpcd.conf /etc/dhcpcd.conf.tmp
        echo "Backup /etc/dhcpcd.conf: /etc/dhcpcd.conf.tmp"
    fi

    # Modify /etc/dhcpcd.conf for network behavior
    if [ "$static_ip" != "NONE" ]; then
        sudo echo "interface $interface" >> /etc/dhcpcd.conf
        sudo echo "static ip_address=$static_ip" >> /etc/dhcpcd.conf
    else
        echo "dhcpcd.conf not changed"
    fi
}

UpdateCodePack ()
{
    # Check Internet Connection
    printf "%s" "Internet connecting..."
    while ! ping -w 1 -c 1 -n 168.95.1.1 &> /dev/null
    do
        printf "%c" "."
    done
    printf "\n%s\n" "Internet connected."

    # Check pwd
    if [ "$PWD" == "$target_dir" ]
    then
        echo "In $target_dir"
    else
        if ls $target_dir &> /dev/null
        then
            cd $target_dir
            echo "Change directory: $PWD"
        else
            echo "ros2_docker path error. Please copy ros2_docker directory under $HOME"
            exit 1
        fi
    fi
    # pwd in ~/ros2_docker

    # Check git
    if [ -x "$(command -v git)" ]; then
        echo "Found git." && git --version
    else
        echo "No git. Installing git..."
        # sudo apt install git -y
    fi

    # Check git control
    if git status &> /dev/null
    then
        echo "git control checked."
    else
        echo "git control not found. \
Delete ros2_docker directory and run \
'cd ~ && git clone https://github.com/cocobird231/RobotVehicle-1.0-ROS2-rpi_sensors.git ros2_docker' \
to grab git controlled directory."
        exit 1
    fi

    # Update submodules
    git submodule update --remote --recursive --force

    # Check previous module setting
    if cat .modulename &> /dev/null
    then
        pack_name=$(cat .modulename)
        echo "Found module name: $pack_name"
    else
        echo ".modulename not found. Run install.sh and select number to install module."
        exit 1
    fi

    if cat .moduleinterface &> /dev/null
    then
        interface=$(cat .moduleinterface)
        echo "Found module interface: $interface"
    else
        echo ".moduleinterface not found. Run install.sh and select number to install module."
        exit 1
    fi
    
    if cat .moduleip &> /dev/null
    then
        static_ip=$(cat .moduleip)
        echo "Found module ip: $static_ip"
    else
        echo ".moduleip not found. Run install.sh and select number to install module."
        exit 1
    fi

    # Update module
    InstallDockerfile
}

## Install Menu
echo "################################################"
printf "\t%s\n\n" "Raspberry Pi Sensor Package Installer"
echo "1) GPS module (MAX-M8Q GNSS, ZED-F9P HAT)"
echo "2) SenseHat module (IMU and environment sensors)"
echo "3) RF Communication module (send and receive)"
echo "4) Ultrasound module (HC-SR04 sensors)"
echo "5) Webcam module (based on OpenCV4)"
echo "u) Update module (git control required)"
echo "q) Exit"
echo "################################################"
echo "Enter number for module installation. Enter 'u' for module update or 'q' to exit:"
read selectNum

if [ "$selectNum" == "1" ]
then
    echo "Install GPS module..."
    pack_name="py_gps"
elif [ "$selectNum" == "2" ]
then
    echo "Install SenseHat module..."
    pack_name="py_sense"
elif [ "$selectNum" == "3" ]
then
    echo "Install RF Communication module..."
    pack_name="py_singlerf"
elif [ "$selectNum" == "4" ]
then
    echo "Install Ultrasound module..."
    pack_name="py_ultrasound"
elif [ "$selectNum" == "5" ]
then
    echo "Install Webcam module..."
    pack_name="cpp_webcam"
elif [ "$selectNum" == "u" ]
then
    echo "Updating module..."
    pack_name="NONE"
    UpdateCodePack
    pack_name="NONE"
else
    pack_name="NONE"
fi

if [ "$pack_name" != "NONE" ]
then
    echo "Preparing package..."
    PreparePackage
    InstallDockerfile
    EnvSetting
else
    echo "Process ended."
fi
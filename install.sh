#!/usr/bin/bash
target_dir="$HOME/ros2_docker"

PARSER_UPDATE="NONE"
PARSER_INSTALL="NONE"
pack_name="NONE"
static_ip="NONE"
interface="eth0"
non_docker="FALSE"

while [[ $# -gt 0 ]]; do
    case $1 in
        -i|--install)
            PARSER_INSTALL="install"
            pack_name="$2"
            # Check is docker
            if [ "$pack_name" == "webrtc" ]
            then
                non_docker="TRUE"
            fi
            shift # past argument
            shift # past value
            ;;
        --interface)
            interface="$2"
            shift # past argument
            shift # past value
            ;;
        --ip)
            static_ip="$2"
            shift # past argument
            shift # past value
            ;;
        --remove)
            PARSER_INSTALL="remove"
            shift # past argument
            ;;
        --forced_update)
            PARSER_UPDATE="forced_update"
            shift # past argument
            ;;
        --preserved_update)
            PARSER_UPDATE="preserved_update"
            shift # past argument
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

CheckParser ()
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

    # Update
    if [ "$PARSER_UPDATE" == "forced_update" ]
    then
        CheckCurrentModule
        git submodule update --init --remote --recursive --force
        InstallScript
    elif [ "$PARSER_UPDATE" == "preserved_update" ]
    then
        CheckCurrentModule
        cp codePack/$pack_name/launch/common.yaml common.yaml.tmp
        git submodule update --init --remote --recursive --force
        mv common.yaml.tmp codePack/$pack_name/launch/common.yaml
        InstallScript
    fi

    if [ "$PARSER_INSTALL" == "remove" ]
    then
        # Get current module info
        # CheckCurrentModule
        # Install
        # InstallScript
        # Environment setting
        # EnvSetting
        Remove
    elif [ "$PARSER_INSTALL" == "install" ]
    then
        # Save module info
        SaveCurrentModule
        # Install
        InstallScript
        # Environment setting
        EnvSetting
    fi
}

CheckCurrentModule ()
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
}

SaveCurrentModule ()
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

    # Store selected module name, interface and ip into files
    touch .modulename
    echo $pack_name > .modulename
    touch .moduleinterface
    echo $interface > .moduleinterface
    touch .moduleip
    echo $static_ip > .moduleip
}

PreparePackage ()
{
    echo "===Prepare Packages==="
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

    # Save module info
    SaveCurrentModule
}

InstallScript ()
{
    echo "===Install Process==="
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

    # Recover run.sh if .tmp exist
    if cat run.sh.tmp &> /dev/null
    then
        cp run.sh.tmp run.sh
        echo "run.sh recovered"
    else
        cp run.sh run.sh.tmp
        echo "Backup run.sh: run.sh.tmp"
    fi

    if [ "$non_docker" == "TRUE" ]
    then
        InstallNonDocker
    else
        InstallDocker
    fi
}

# Must have install.sh script located at $target_dir/codePack/$pack_name/install.sh
InstallNonDocker()
{
    echo "===Install Process==="
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

    rm -rf run.sh && ln codePack/$pack_name/run.sh run.sh
    sudo chmod a+x run.sh

    # Required environment installation and update
    sudo apt update
    sudo apt install python3 python3-dev python3-pip git curl -y

    # Install the requirement files linked from each module's code pack
    xargs sudo apt install < requirement_apt.txt
    python3 -m pip install requirement_pip.txt

    # Run install.sh script
    sudo chmod a+x codePack/$pack_name/install.sh
    ./codePack/$pack_name/install.sh $PWD/codePack/$pack_name
}

InstallDocker()
{
    echo "===Install Process==="
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

    # Link requirement file to ~/ros2_docker for Dockerfile installation
    rm -rf requirement_apt.txt && ln codePack/$pack_name/requirement_apt.txt requirement_apt.txt
    rm -rf requirement_pip.txt && ln codePack/$pack_name/requirement_pip.txt requirement_pip.txt
    rm -rf source_env.txt && ln codePack/$pack_name/source_env.txt source_env.txt

    # Link common.yaml file to ~/ros2_docker for convenient modifying
    rm -rf common.yaml && ln codePack/$pack_name/launch/common.yaml common.yaml

    # Modify run.sh by adding specific $pack_name source_env.txt
    cat source_env.txt >> run.sh
    sudo chmod a+x run.sh
    # Add docker run process
    echo "sudo docker run -v ~/ros2_docker/codePack/$pack_name/launch/common.yaml:/ros2_ws/install/$pack_name/share/$pack_name/launch/common.yaml --rm --privileged --net host -it ros2_docker ros2 launch $pack_name launch.py" >> run.sh

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
    echo "===Environment Setting==="
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
    echo "===Update Process==="
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
'cd ~ && git clone https://github.com/davidweitaiwan/RV-1.0-rpi_sensors-install.git ros2_docker' \
to grab git controlled directory."
        exit 1
    fi

    # Ask if preserve common.yaml file
    read -p "Preserve current common.yaml file ?(y/n):" selectNum
    if [ "$selectNum" == "y" ]
    then
        # Check previous module setting
        if cat .modulename &> /dev/null
        then
            pack_name=$(cat .modulename)
            cp codePack/$pack_name/launch/common.yaml common.yaml.tmp
        else
            echo ".modulename not found. common.yaml will not preserved."
            selectNum="n"
        fi
    fi

    # Update submodules
    git submodule update --remote --recursive --force

    # Recovering common.yaml
    if [ "$selectNum" == "y" ]
    then
        mv common.yaml.tmp codePack/$pack_name/launch/common.yaml
        echo "common.yaml recovered."
    fi
}

Remove ()
{
    echo "===Remove Process==="
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
    
    # Target files
    rm -rf requirement_apt.txt
    rm -rf requirement_pip.txt
    rm -rf source_env.txt
    rm -rf common.yaml
    rm -rf ros2_docker.desktop.tmp
    rm -rf .module*
    
    # Recover Dockerfile if .tmp exist
    if cat Dockerfile.tmp &> /dev/null
    then
        mv Dockerfile.tmp Dockerfile
        echo "Dockerfile recovered"
    fi
    
    # Recover run.sh if .tmp exist
    if cat run.sh.tmp &> /dev/null
    then
        mv run.sh.tmp run.sh
        echo "run.sh recovered"
    fi
    
    # System files
    sudo rm -rf /etc/xdg/autostart/ros2_docker.desktop
    
    # Recover /boot/config.txt if .tmp exist
    if sudo cat /boot/config.txt.tmp &> /dev/null
    then
        sudo mv /boot/config.txt.tmp /boot/config.txt
        echo "/boot/config.txt recovered"
    fi
    
    # Recover /etc/dhcpcd.conf if .tmp exist
    if sudo cat /etc/dhcpcd.conf.tmp &> /dev/null
    then
        sudo mv /etc/dhcpcd.conf.tmp /etc/dhcpcd.conf
        echo "/etc/dhcpcd.conf recovered"
    fi
    exit 0
}

CheckParser
if [ "$pack_name" != "NONE" ]
then
    exit 0
fi

## Install Menu
echo "################################################"
printf "\t%s\n\n" "Raspberry Pi Sensor Package Installer"
echo "1) GPS module (MAX-M8Q GNSS, ZED-F9P HAT)"
echo "2) SenseHat module (IMU and environment sensors)"
echo "3) RF Communication module (send and receive)"
echo "4) Ultrasound module (HC-SR04 sensors)"
echo "5) Webcam module (based on OpenCV4)"
echo "6) WebRTC module (GStreamer)"
echo "u) Update codePack (git control required)"
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
elif [ "$selectNum" == "6" ]
then
    echo "Install WebRTC module..."
    pack_name="webrtc"
    non_docker="TRUE"
elif [ "$selectNum" == "u" ]
then
    echo "Updating module..."
    pack_name="NONE"
    UpdateCodePack
    CheckCurrentModule
    InstallScript
    pack_name="NONE"
else
    pack_name="NONE"
fi

if [ "$pack_name" != "NONE" ]
then
    echo "Preparing package..."
    PreparePackage
    InstallScript
    EnvSetting
else
    echo "Process ended."
fi

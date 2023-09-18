#!/usr/bin/bash
target_dir="$HOME/ros2_docker"

PARSER_USED="FALSE" # Is parser used
PARSER_REMOVE="FALSE" # Remove current program and settings
PARSER_UPDATE="FALSE" # Update codePack, no install
PARSER_INSTALL="FALSE" # Install program from codePack

preserve_conf="FALSE" # Preserve current common.yaml file while installing
pack_name="NONE"
static_ip="NONE"
interface="eth0"
non_docker="FALSE"


# Parser process orders
# 1.    PARSER_REMOVE       remove program under ros2_ws and environment settings
# 2.    PARSER_UPDATE       update codePack without installation
# 3.    PARSER_INSTALL      install program from codePack to ros2_ws
#       --preserve          preserve common.yaml under ros2_ws while installing
#       --interface         set network interface
#       --ip                set static ip (not supported)

while [[ $# -gt 0 ]]; do
    case $1 in
        -rm|--remove) # Can be worked independently
            PARSER_REMOVE="TRUE"
            PARSER_USED="TRUE"
            shift # past argument
            ;;
        -u|--update) # Can be worked independently
            PARSER_UPDATE="TRUE"
            PARSER_USED="TRUE"
            shift # past argument
            ;;
        -i|--install) # Can be worked independently
            PARSER_INSTALL="TRUE"
            PARSER_USED="TRUE"
            pack_name="$2" # <pack_name> or auto
            # Check is docker
            if [ "$pack_name" == "webrtc" ]
            then
                non_docker="TRUE"
            fi
            shift # past argument
            shift # past value
            ;;
        -p|--preserve)
            preserve_conf="TRUE"
            shift # past argument
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
        -*|--*)
            echo "Unknown option $1"
            exit 0
            ;;
    *)
      shift # past argument
      ;;
  esac
done

function PrintError () # Red
{
    error_color="\033[1;91m"
    reset_color="\033[0m"
    printf "${error_color}%s${reset_color}\n" "$1"
}

function PrintSuccess () # Green
{
    success_color="\033[1;92m"
    reset_color="\033[0m"
    printf "${success_color}%s${reset_color}\n" "$1"
}

function PrintWarning () # Yellow
{
    warn_color="\033[1;93m"
    reset_color="\033[0m"
    printf "${warn_color}%s${reset_color}\n" "$1"
}

vercomp ()
{
    if [[ $1 == $2 ]]
    then
        return 0
    fi
    local IFS=.
    local i ver1=($1) ver2=($2)
    # fill empty fields in ver1 with zeros
    for ((i=${#ver1[@]}; i<${#ver2[@]}; i++))
    do
        ver1[i]=0
    done
    for ((i=0; i<${#ver1[@]}; i++))
    do
        if [[ -z ${ver2[i]} ]]
        then
            # fill empty fields in ver2 with zeros
            ver2[i]=0
        fi
        if ((10#${ver1[i]} > 10#${ver2[i]}))
        then
            return 1
        fi
        if ((10#${ver1[i]} < 10#${ver2[i]}))
        then
            return 2
        fi
    done
    return 0
}

CheckParser ()
{
    # Check pwd
    CheckTargetPath

    # Check remove
    if [ "$PARSER_REMOVE" == "TRUE" ]
    then
        Remove
    fi

    # Check update
    if [ "$PARSER_UPDATE" == "TRUE" ]
    then
        UpdateCodePack
    fi

    # Check install
    if [ "$PARSER_INSTALL" == "TRUE" ]
    then
        if [ "$pack_name" == "auto" ]
        then
            CheckCurrentModule
            if [[ $? -eq 1 ]]
            then
                Remove
                return 1
            fi
        fi

        InstallScript # Return 0 if succeed
        if [[ $? -eq 0 ]]
        then
            EnvSetting
            SaveCurrentModule
        else
            Remove
            return 1
        fi
    fi
}

# cd into $target_dir
CheckTargetPath ()
{
    if [ "$PWD" == "$target_dir" ]
    then
        echo "In $target_dir"
    else
        if ls $target_dir &> /dev/null
        then
            cd $target_dir
            echo "Change directory: $PWD"
        else
            PrintError "Target path error: $target_dir"
            exit 1
        fi
    fi
}

# Will set $pack_name, $interface and $static_ip
CheckCurrentModule ()
{
    # Check pwd
    CheckTargetPath

    # Check previous module setting
    if cat .modulename &> /dev/null
    then
        pack_name=$(cat .modulename)
        echo "Found module name: $pack_name"
    else
        PrintError ".modulename not found. Run install.sh and select number to install module."
        return 1
    fi

    if cat .moduleinterface &> /dev/null
    then
        interface=$(cat .moduleinterface)
        echo "Found module interface: $interface"
    else
        PrintError ".moduleinterface not found. Run install.sh and select number to install module."
        return 1
    fi
    
    if cat .moduleip &> /dev/null
    then
        static_ip=$(cat .moduleip)
        echo "Found module ip: $static_ip"
    else
        PrintError ".moduleip not found. Run install.sh and select number to install module."
        return 1
    fi
    return 0
}

# Will create .modulename, .moduleinterface and .moduleip
SaveCurrentModule ()
{
    # Check pwd
    CheckTargetPath

    # Store selected module name, interface and ip into files
    touch .modulename
    echo $pack_name > .modulename
    touch .moduleinterface
    echo $interface > .moduleinterface
    touch .moduleip
    echo $static_ip > .moduleip
}

InstallScript ()
{
    echo "===Install Process==="

    # Check pwd
    CheckTargetPath

    # Check $pack_name available
    if ls codePack/$pack_name &> /dev/null
    then
        echo "Package found: $pack_name"
    else
        return 1
    fi

    # Recover run.sh if .tmp exist
    if cat run.sh.tmp &> /dev/null
    then
        cp run.sh.tmp run.sh
        echo "run.sh recovered"
    else
        cp run.sh run.sh.tmp
        echo "Backup run.sh: run.sh.tmp"
    fi

    # Prevent non-exist files
    touch requirement_apt.txt
    touch requirement_pip.txt
    touch source_env.txt

    # Copy requirement files to ~/ros2_docker for Dockerfile installation
    if ls codePack/$pack_name/requirement_apt.txt &> /dev/null
    then
        cp codePack/$pack_name/requirement_apt.txt requirement_apt.txt
    fi

    if ls codePack/$pack_name/requirement_pip.txt &> /dev/null
    then
        cp codePack/$pack_name/requirement_pip.txt requirement_pip.txt
    fi

    if ls codePack/$pack_name/source_env.txt &> /dev/null
    then
        cp codePack/$pack_name/source_env.txt source_env.txt
    fi

    if ! cat common.yaml &> /dev/null
    then
        preserve_conf="FALSE"
    fi

    # Copy new common.yaml to $target_dir if --preserve not set
    if [ "$preserve_conf" == "FALSE" ]
    then
        cp codePack/$pack_name/launch/common.yaml common.yaml
    fi

    # Modify run.sh by adding specific $pack_name source_env.txt
    cat source_env.txt >> run.sh

    # Required environment installation and update
    sudo apt update
    sudo apt install python3 python3-dev python3-pip git curl -y

    # Install the requirement files linked from each module's code pack
    xargs sudo apt install -y < requirement_apt.txt
    python3 -m pip install -r requirement_pip.txt

    # Start installation
    if [ "$non_docker" == "TRUE" ]
    then
        InstallNonDocker
    else
        InstallDocker
    fi

    # Check pwd
    CheckTargetPath

    rm -rf requirement_apt.txt
    rm -rf requirement_pip.txt
    rm -rf source_env.txt
}

# Must have install.sh script located at $target_dir/codePack/$pack_name/install.sh
InstallNonDocker()
{
    echo "===Install Process==="

    # Check pwd
    CheckTargetPath

    # Run install.sh script
    sudo chmod a+x codePack/$pack_name/install.sh
    ./codePack/$pack_name/install.sh $PWD/codePack/$pack_name

    cp $target_dir/codePack/$pack_name/run.sh $target_dir
}

InstallDocker()
{
    echo "===Install Process==="

    # Check pwd
    CheckTargetPath

    # Add docker run process
    echo "sudo docker run -v ~/ros2_docker/common.yaml:/ros2_ws/install/$pack_name/share/$pack_name/launch/common.yaml -v ~/ros2_docker/launch/qos:/ros2_ws/launch/qos --rm --privileged --net host -it ros2_docker ros2 launch $pack_name launch.py" >> run.sh
    sudo chmod a+x run.sh

    ## Install Dockerfile
    ################################################################

    # Install Dockerfile Process
    echo "Installing dockerfile..."

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
    echo "RUN . /opt/ros/\${ROS_DISTRO}/setup.sh && colcon build --packages-select $pack_name vehicle_interfaces" >> Dockerfile
    
    # Dockerfile Installation
    sudo docker build -t ros2_docker .
}

# Create ros2_docker.desktop under /etc/xdg/autostart
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

# Update codePack
UpdateCodePack ()
{
    echo "===Update Process==="

    # Check pwd
    CheckTargetPath

    # Check Internet Connection
    printf "%s" "Internet connecting..."
    while ! ping -w 1 -c 1 -n 168.95.1.1 &> /dev/null
    do
        printf "%c" "."
    done
    printf "\n%s\n" "Internet connected."

    # Check git
    if [ -x "$(command -v git)" ]; then
        echo "Found git." && git --version
    else
        echo "No git. Installing git..."
        sudo apt install git -y
    fi

    # Check git control
    if git status &> /dev/null
    then
        echo "git control checked."
    else
        PrintError "git control not found. \
Delete ros2_docker directory and run \
'cd ~ && git clone https://github.com/davidweitaiwan/RV-1.0-rpi_sensors-install.git ros2_docker' \
to grab git controlled directory."
        return 1
    fi

    # Update submodules
    git submodule update --remote --recursive --force
}

# Remove common.yaml, .txt, .tmp, .module* files and /etc/xdg/autostart/ros2_docker.desktop
Remove ()
{
    echo "===Remove Process==="

    # Check pwd
    CheckTargetPath
    
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
}

PreparePackage ()
{
    echo "===Prepare Packages==="
    CheckTargetPath

    ## Install Menu
    echo "################################################"
    printf "\t%s\n\n" "Raspberry Pi Sensor Package Installer"
    echo "1) GPS module (MAX-M8Q GNSS, ZED-F9P HAT)"
    echo "2) SenseHat module (IMU and environment sensors)"
    echo "3) RF Communication module (send and receive)"
    echo "4) Ultrasound module (HC-SR04 sensors)"
    echo "5) Webcam module (based on OpenCV4)"
    echo "6) WebRTC module (GStreamer)"
    echo "u) Update package (git control required)"
    echo "r) Remove package"
    echo "q) Exit"
    echo "################################################"
    read -p "Enter number for module installation. Enter 'u' for package update, 'r' for package removal or 'q' to exit:" selectNum

    if [ "$selectNum" == "q" ]
    then
        return 0
    elif [ "$selectNum" == "r" ]
    then
        Remove
        return 0
    elif [ "$selectNum" == "u" ]
    then
        CheckCurrentModule
        if [[ $? -eq 1 ]]
        then
            PrintError "[PreparePackage] CheckCurrentModule failed. Exiting..."
            return 1
        fi
        UpdateCodePack
        InstallScript # Return 0 if succeed
        if [[ $? -eq 0 ]]
        then
            EnvSetting
            SaveCurrentModule
        else
            Remove
            return 1
        fi
    elif [ "$selectNum" == "1" ]
    then
        echo "[PreparePackage] Install GPS module..."
        pack_name="py_gps"
    elif [ "$selectNum" == "2" ]
    then
        echo "[PreparePackage] Install SenseHat module..."
        pack_name="py_sense"
    elif [ "$selectNum" == "3" ]
    then
        echo "[PreparePackage] Install RF Communication module..."
        pack_name="py_singlerf"
    elif [ "$selectNum" == "4" ]
    then
        echo "[PreparePackage] Install Ultrasound module..."
        pack_name="py_ultrasound"
    elif [ "$selectNum" == "5" ]
    then
        echo "[PreparePackage] Install Webcam module..."
        pack_name="cpp_webcam"
    elif [ "$selectNum" == "6" ]
    then
        echo "[PreparePackage] Install WebRTC module..."
        pack_name="webrtc"
        non_docker="TRUE"
    else
        PrintError "[PreparePackage] Unknown input number. Exiting..."
        return 1
    fi

    # Network Interface Selection
    read -p "Enter network interface (default eth0):" interface
    if [ $interface ]
    then
        echo "Interface: $interface"
    else
        interface="eth0"
        echo "Default interface: $interface"
    fi

    # Network IP selection
    read -p "Use DHCP? (y/n):" static_ip
    if [[ "$static_ip" != "n" && "$static_ip" != "N" ]]
    then
        static_ip="NONE"
    else
        read -p "Enter static ip (ex 192.168.3.100/16):" static_ip
        if [ ! $static_ip ]
        then
            static_ip="NONE"
        fi
    fi
    echo "Static IP: $static_ip"

    # Remove old package
    Remove

    # Install package
    InstallScript # Return 0 if succeed
    if [[ $? -eq 0 ]]
    then
        EnvSetting
        SaveCurrentModule
    else
        Remove
        return 1
    fi
}

entry_pwd="$PWD"

if [ "$PARSER_USED" == "TRUE" ]
then
    CheckParser
else
    PreparePackage
fi

if [ "$PWD" != "$entry_pwd" ]
then
    if ls $entry_pwd &> /dev/null
    then
        cd $entry_pwd
        echo "Change directory: $PWD"
    fi
fi
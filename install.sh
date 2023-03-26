#!/usr/bin/bash
DRY_RUN=NO
while [[ $# -gt 0 ]]; do
  case $1 in
    -e|--extension)
      EXTENSION="$2"
      shift # past argument
      shift # past value
      ;;
    --dry-run)
      DRY_RUN=YES
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

PreparePackage ()
{
    rm -rf ros2_docker
    mkdir -p ros2_docker/code

    cp -rv codePack/vehicle_interfaces ros2_docker/code/vehicle_interfaces
    cp -rv codePack/$PACKNAME ros2_docker/code/$PACKNAME

    cp ros_entrypoint.sh ros2_docker/ros_entrypoint.sh
    cp install_docker.sh ros2_docker/install_docker.sh
    cp Dockerfile ros2_docker/Dockerfile
    cp run.sh ros2_docker/run.sh

    cp codePack/$PACKNAME/requirement_apt.txt ros2_docker/requirement_apt.txt
    cp codePack/$PACKNAME/requirement_pip.txt ros2_docker/requirement_pip.txt
    cp codePack/$PACKNAME/source_env.txt ros2_docker/source_env.txt
    line_default=20
    sed -i "${line_default}r ros2_docker/source_env.txt" ros2_docker/run.sh
    echo "" >> ros2_docker/run.sh
    echo "sudo docker run -v ~/ros2_docker/code/$PACKNAME/launch/common.yaml:/ros2_ws/install/$PACKNAME/share/$PACKNAME/launch/common.yaml --rm --privileged --net host -it ros2_docker ros2 launch $PACKNAME launch.py" >> ros2_docker/run.sh

    if [ $DRY_RUN != YES ]
    then
        mv ros2_docker ~/ros2_docker
    fi
}

InstallDockerfile ()
{
    if [ $DRY_RUN != YES ]
    then
        cd ~/ros2_docker
    else
        cd ros2_docker
    ln code/$PACKNAME/launch/common.yaml common.yaml
    fi
    echo "PWD: $PWD"
    
    echo "Enter network interface (default eth0):"
    read interface
    if [ $interface ]
    then
        echo "Interface: $interface"
    else
        interface="eth0"
        echo "Default interface: $interface"
    fi

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

    echo "Installing dockerfile..."
    chmod a+x ./install_docker.sh
    if [ $DRY_RUN == YES ]
    then
        if [ "$static_ip" == "NONE" ]
        then
            echo "Install with interface: $interface with DHCP"
            ./install_docker.sh -i $interface --dry-run
        else
            echo "Install with interface: $interface with IP: $static_ip"
            ./install_docker.sh -i $interface --ip $static_ip --dry-run
        fi
        cd ..
        rm -rf ros2_docker
    else
        if [ "$static_ip" == "NONE" ]
        then
            echo "Install with interface: $interface with DHCP"
            ./install_docker.sh -i $interface
        else
            echo "Install with interface: $interface with IP: $static_ip"
            ./install_docker.sh -i $interface --ip $static_ip
        fi
    fi
}

## Install Menu
echo "################################################"
printf "\t\t%s\n\n" "Select Install Package"
echo "1) GPS module (MAX-M8Q GNSS HAT)"
echo "2) SenseHat module (IMU and environment sensors)"
echo "3) RF Communication module (send and receive)"
echo "4) Ultrasound module (Three HC-SR04 sensors)"
echo "5) Webcam module (Based on OpenCV4)"
echo "q) Exit"
echo "################################################"
echo "Enter the install package number or press q to exit:"
read selectNum

if [ "$selectNum" == "1" ]
then
    echo "Install GPS module..."
    PACKNAME="py_gps"
elif [ "$selectNum" == "2" ]
then
    echo "Install SenseHat module..."
    PACKNAME="py_sense"
elif [ "$selectNum" == "3" ]
then
    echo "Install RF Communication module..."
    PACKNAME="py_singlerf"
elif [ "$selectNum" == "4" ]
then
    echo "Install Ultrasound module..."
    PACKNAME="py_ultrasound"
elif [ "$selectNum" == "5" ]
then
    echo "Install Webcam module..."
    PACKNAME="cpp_webcam"
else
    PACKNAME="NONE"
fi

if [ "$PACKNAME" != "NONE" ]
then
    echo "Preparing package..."
    PreparePackage
    InstallDockerfile
else
    echo "Process ended."
fi
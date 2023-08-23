# ROS2-IMU
## Overview
This document shows how to run a C++ ROS2 node that reads data from an IMU sensor. It also showcases the IMUD daemon/library sample code so you can build your own IMU node.
- sensor-daemon: the IMUD service to access IMU data from DSP.
- sensor-client: the library for communicating with IMUD.
- imu-ros2node: the sample implementation of IMU node based on ROS2.
**TIP:** All these components are built-in in RB5 image as binary programs.

Follow these pre-requisite steps to setup ROS on Qualcomm Robotics RB5, once you are connected to a network
1 Download and Install ROS2 
The built-in IMU ROS node supports ROS2 Dashing only.
**1.1 Connect Qualcomm Robotics RB5 with adb**
adb shell
**1.2 Install ROS2 via Debian packages**
You can reference the official ROS document:
https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Debians.html
Or you can directly use the commands below:
# update locales
apt update && apt install locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# add apt source
apt update && apt install curl gnupg2 lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt update

# install ROS2 package
apt install ros-dashing-ros-base
Wait until all ROS2 packages are installed completely.
2 Verify the ROS2 installation**
source /opt/ros/dashing/setup.bash
ros2
Will show the ros2 help message if installation is complete.
## 3 Run IMU ROS node
**3.1 Clone the sample app and re-build ROS2 to include IMU Node**
In terminal 1  

adb shell
cd /data
 
git clone https://github.com/quic/sample-apps-for-robotics-platforms.git

cd ~ && mkdir ~/ros_ws
 
mv <path to directory in Git repository>/imu-ros2node ~/ros_ws/


# Install colcon
apt install python3-colcon-common-extensions

# Re-build ROS 
    cd ~/ros_ws
    source /opt/ros/dashing/setup.bash
    colcon build

    cd ~/ros_ws
    . install/local_setup.sh
**3.2 Run the IMU ROS node sample *
    
In terminal 1, run
Source ~/.bashrc
ros2 run imu-ros2node imu-ros2node

In terminal 2 , Run

source /opt/ros/dashing/setup.bash
    
    ros2 topic echo /imu


Pick up RB5 and shake it, you could see the IMU data are changing in the logs:
## License
This is licensed under the BSD 3-clause-Clear “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.




imu-ros2node is a ROS2 node for publishing IMU sensor data to ROS2 topic.

Development Preparation:
    1. Install ROS2 Dashing+. Reference: https://index.ros.org/doc/ros2/Installation/
    2. Install colcon: apt install python3-colcon-common-extensions

Create Workspace:
    mkdir ~/ros_ws
    mv imu-ros2node ~/ros_ws/

Build Code:
    cd ~/ros_ws
    source /opt/ros/dashing/setup.bash
    colcon build

Install:
    cd ~/ros_ws
    . install/local_setup.sh

Run:
    ros2 run imu-ros2node imu-ros2node

Test:
    ros2 run imu-ros2node imu-ros2test
    # or
    ros2 topic echo /imu

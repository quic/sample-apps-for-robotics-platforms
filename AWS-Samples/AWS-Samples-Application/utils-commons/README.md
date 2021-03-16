### Introduction
This is the common library for all of AWS RoboMaker packages and does not require any permissions to be set.

1. Clone the application

    i.Create a ROS workspace and a source directory
    ```
    mkdir -p ~/ros-workspace/src 
    cd ~/ros-workspace/src
    ```
    ii. Clone the utils-common app
    ```
    git clone https://github.com/aws-robotics/utils-common.git -b release-latest
    cd ~/ros-workspace 
    apt-get update && rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```
2. Build the utils-common app
    ```
    source  /opt/ros/dashing/setup.bash
    cd ~/ros-workspace && colcon build
    source ~/ros-workspace/install/local_setup.bash
    ```
3. Execution outputs of the utils-common app
    
    ![AWS Dashboard](image/UtilsCommon_Screenshot_1.PNG)

For more information on colcon bundle and bundle installation please read:
 https://github.com/aws-robotics/utils-common




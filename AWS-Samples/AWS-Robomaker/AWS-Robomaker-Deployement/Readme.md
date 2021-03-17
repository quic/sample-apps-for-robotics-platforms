### Introduction

This project talks about deployment of robotics applications (ROS) onto Qualcomm RB5 Robotics platform from AWS RoboMaker. AWS RoboMaker is a cloud service that provides a development environment that we can use to develop and deploy ROS applications on physical robots, simulate and test applications (without the need for a real robot). It also provides AWS analytics related ROS packages that one can integrate with their robotic applications for posting data to AWS cloud.
Step-by-step process of AWS RoboMaker application deployment is covered in the blog https://aws.amazon.com/blogs/robotics/deploy-robotic-applications-using-aws-robomaker/ (Robomaker Blog). We were able to deploy to Qualcomm RB5 following this content. The cross-compilation and bundling steps detailed in the blog link above are for RPi3 which is based on Armhf architecture. Since RB5 is based on ARM64 architecture we will describe cross-compilation Dockerfile for ARM64 based SBCs and show you how to install ARM64 packages for bundling applications using colcon.
We will also show you a neat trick that makes final application deployment step hassle-free.
The pre-requisites before you try application deployment from RoboMaker:
1.	Turtlebot burger is connected to RB5
2.	Java8 is installed
3.	ARM64 based AWS Greengrass installed
4.	Start running the steps described in the link above, RoboMaker Blog
5.	Create a new development environment on RoboMaker choosing “ROS2 Dashing (beta)” in the field “Pre-installed ROS Distribution”
**Step 14** in the Robomaker blog specifically talks about cross-compilation and bundling. Here are the steps mentioned in the blog for ARMHF. These steps need to be modified for other platforms/architecture:
```
cd /opt/robomaker/cross-compilation-dockerfile/
sudo bin/build_image.bash
cd ~/environment/HelloWorld/robot_ws
sudo docker run -v $(pwd):/ws -it ros-cross-compile:armhf
cd ws
apt update
rosdep install --from-paths src --ignore-src -r -y
colcon build –build-base armhf_build –install-base armhf_install
colcon bundle –build-base armhf_build –install-base armhf_install –bundle-base armhf_bundle –apt-sources-list /opt/cross/apt-sources.yaml
exit
```
In case lidar is not used, disable in robot.launch.py by commenting following lines as follows:

```
#lidar_pkg_dir = LaunchConfiguration(
#        'lidar_pkg_dir',
#        default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))


#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource([lidar_pkg_dir, '/hlds_laser.launch.py']),
#            launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
#        ),
```
Here are the list of steps for RB5 which is based on ARM64 architecture (Ubuntu-18.04, bionic):

```
sudo apt install qemu-user-static
sudo cp /usr/bin/qemu-aarch64-static /opt/robomaker/cross-compilation-dockerfile/
cd /opt/robomaker/cross-compilation-dockerfile/
mv Dockerfile Dockerfile.armhf
```

We use a modified version of the original ARMHF Dockerfile for ARM64. Please create a new file with a filename ‘Dockerfile’ and copy the below contents into it: 

```
ARG ROS_VERSION=dashing
ARG UBUNTU_VERSION=bionic
#we start at ubuntu-18.04 (bionic) with ROS dashing preinstalled as our base image
FROM arm64v8/ros:dashing-ros-base-bionic

ENV PATH="/root/.local/bin:${PATH}"

# Copy qemu-aarch64-static from the host machine. This will be our interpreter for anything and everything that runs inside this container. As our host is x86 we need ARM64->x86 interpreter
COPY qemu-aarch64-static /usr/bin

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Package manager automatically installs xenial ARM64 packages (base image effect)
# Installed for convenience
RUN apt-get update && apt-get install -y vim

# Add raspicam_node sources to apt
#RUN apt-get update && apt-get install -y apt-transport-https \
#    && echo "deb https://packages.ubiquityrobotics.com/ubuntu/ubiquity xenial main" > /etc/apt/sources.list.d/ubiquity-latest.list \
#    && apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C3032ED8

# Install Raspberry Pi package sources and libraries
#RUN apt-get update && apt-get install -y gnupg lsb-release software-properties-common \
#    && add-apt-repository -y ppa:ubuntu-pi-flavour-makers/ppa \
#    && apt-get update && apt-get install -y libraspberrypi0

# Install Python and colcon
RUN apt-get update && apt-get install -y \
      python \
      python3-apt \
      curl \
    && curl -O https://bootstrap.pypa.io/get-pip.py \
    && python3 get-pip.py \
    && python2 get-pip.py \
    && python3 -m pip install -U colcon-ros-bundle

# Add custom rosdep rules
#COPY custom-rosdep-rules/raspicam-node.yaml /etc/ros/rosdep/custom-rules/raspicam-node.yaml
#RUN echo "yaml file:/etc/ros/rosdep/custom-rules/raspicam-node.yaml" > /etc/ros/rosdep/sources.list.d/22-raspicam-node.list \
RUN echo "yaml https://s3-us-west-2.amazonaws.com/rosdep/python.yaml" > /etc/ros/rosdep/sources.list.d/18-aws-python.list \
    && rosdep update

# Add custom pip rules
COPY custom-pip-rules.conf   /etc/pip.conf

# Add custom apt sources for bundling
COPY bionic-sources-arm64.yaml /opt/cross/apt-sources.yaml
```                                                                       


Please save the following contents into a file called bionic-sources-arm64.yaml. This lists ARM64 and ROS2 packages for bundling the robot application:

```
# ARM Support

# ARM Support
deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ bionic main restricted universe multiverse
deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ bionic-updates main restricted universe multiverse
deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ bionic-backports main restricted
deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ bionic-security main restricted universe multiverse
# ROS
deb [arch=arm64] http://packages.ros.org/ros2/ubuntu bionic main
```

Once the Dockerfile is saved, we continue with following steps:
```
sudo docker build -t ros-cross-compile:arm64 .
cd ~/environment/HelloWorld/robot_ws
sudo docker run -v $(pwd):/ws -it ros-cross-compile:arm64
cd ws
apt update
rosdep install -–from-paths src -–ignore-src -r -y
colcon build
colcon bundle --apt-sources-list /opt/cross/apt-sources.yaml
exit
```
At this stage please switch over and resume from step 15 in the main Robomaker blog. 
In the Robomaker blog section “Create and deploy the robot application with AWS RoboMaker” 16 steps are given. We have observed that for successful robot application deployment AWS Greengrass group settings need to be toggled every time after a deployment is created (Step 16). This involves changing the AWS Greengrass group setting (Default Lambda function containerization) to "Greengrass Container" first and then back to "No Container" and "Redeploy". We have made a video to clearly demonstrate this. Please watch this setting procedure here https://youtu.be/AtCmLLq7q5E.... This video was originally made for RB3 but these steps are applicable for RB5 too.

### Application Execution on RB5

Once the bundle is successfully deployed on RB5 the RoboMaker Dashboard deployment status is updated to “Successfully Deployed”.
The colcon bundle contains complete development environment to execute the ROS sample application including ROS framework, python etc. (in this case Dashing). Go to the folder where bundle was installed. This is usually located under /home/ggc_user/. Every time bundle is deployed AWS GreenGrass creates 2 folders with unique, long names. These folders contain workspace and dependencies related packages. Let us call these folders “dependencies” and “workspace”. “workspace” folder can be identified by checking for the presence of the folder named “opt” the file “setup.sh”. Now run the following steps:  
1.	Folder /home/ggc_user/XXYY/dependencies is created with all the dependencies needed to run ‘ROS application rotate’
2.	Now source your ROS framework environment:
    
    i.	Set environment variable BUNDLE_CURRENT_PREFIX

        export BUNDLE_CURRENT_PREFIX = /home/ggc_user/XXYY/dependencies
    
    ii.	Source setup.sh from dependencies folder

        source /home/ggc_user/XXYY/dependencies/setup.sh
3.	Now overlay application environment on top of framework environment. Folder /home/ggc_user/XXYY/workspace is created with all application related packages:
    I.	Set BUNDLE_CURRENT_PREFIX to the folder for the latest deployment (Please notice that BUNDLE_CURRENT_PREFIX environment variable is being changed again and this is OK).

        export BUNDLE_CURRENT_PREFIX = /home/ggc_user/XXYY/workspace 
    
    II.	Source the setup.sh in /home/ggc_user/XXYY/workspace

        source /home/ggc_user/XXYY/workspace/setup.sh

4.	Now run the turtlebot_bringup application:

    i.	Export environment variables
        
        export TURTLEBOT3_MODEL=YOURROBOTMODEL
    
    ii.	Set the domain id:

        export ROS_DOMAIN_ID=30

    iii. turtlebot_bringup 

        ros2 launch turtlebot3_bringup robot.launch.py

5.	Open a new terminal and run “rotate” application: 

    i.	Repeat steps 1, 2 and 3 above to set the environment
    ii.	Set the domain id:

        export ROS_DOMAIN_ID=30

    iii.	ros2 run hello_world_robot rotate

For more information on colcon bundle and bundle installation please read:
https://github.com/colcon/colcon-bundle

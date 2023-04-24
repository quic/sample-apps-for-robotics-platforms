# Ros_Tensorflow environment set up instructions

  This paper introduces how to integrate Tensorflow framework into ROS with object detection API on RB5 platform. It can locate and recognize multiple objects in a single image. The TensorFlow Object Detection API is an open source framework built on top of TensorFlow that makes it easy to construct, train and deploy object detection models. ROS provides a publish subscribe communication framework to build distributed computing system simply and quickly. Here are the steps to build.

## 1  Set up the ROS environment

**(1) Connect the network and set the source**

```shell
adb shell
su
sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
```

**(2) set the public key and update api**

Execute the apt-get update command that will generate an error, use the error to set the public key. You need to configure it according to your network environment.

```shell
apt-get update
```

![Image text](image/1_apt-get_update_error.png)

```shell
apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F42ED6FBAB17C654
apt-get update
```

![Image text](image/2_apt-get_update_success.png)

**(3) install the ROS**

```shell
apt-get install ros-melodic-desktop-full
```

![Image text](image/3_install-ros-melodic-desktop.png)

**4、Add the environment variable and install dependency package**

```shell
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
apt install ros-melodic-usb-cam
apt install ros-melodic-image-view
```



## 2  Set up the ROS workspace

**（1）Install catkin_pkg**

```SHELL
apt-get install python-pip 
pip install catkin_pkg
```

**（2） Create and compile the workspace**

```shell
mkdir -p ~/catkin_ws/src
cd  ~/catkin_ws
catkin_make
```

![Image text](image/4_compile_the%2Bworkspace.png)



## 3  Install the Tensorflow

**Install the tensorflow and package**
On PC 
(1) Download "tensorflow-1.13.1-cp27-none-linux_aarch64.whl" from URL:https://github.com/lhelontra/tensorflow-on-arm/releases/tag/v1.13.1
(2) adb push tensorflow-1.13.1-cp27-none-linux_aarch64.whl to /data in RB5

On adb shell
```SHELL
adb shell
cd /data
pip install tensorflow-1.13.1-cp27-none-linux_aarch64.whl
pip install matplotlib
apt-get install protobuf-compiler python-pil python-lxml
```



## 4  ros_tensorflow

**1、Grab the source**

```shell
 cd ~/catkin_ws/src
 git clone https://github.com/cong/ros_tensorflow.git"
```

**2、Build**

```shell
cd ~/catkin_ws/
catkin_make
```

**3、connect a panel and a camera**

Connect RB5 and monitor or TV with HDMI cable. Ensure that the monitor or TV must have HDMI interface.Pull the switch to OFF as shown in the following figure.

![Image text](image/5_set_device.png)

```shell
adb reboot bootloader
fastboot oem select-display-panel none
```

Power off and power on the device, open a terminal.

```shell
mkdir -p /usr/bin/weston_socket
export XDG_RUNTIME_DIR=/usr/bin/weston_socket
export LD_LIBRARY_PATH=/usr/lib:/usr/lib/aarch64-linux-gnu/
weston --tty=1 --connector=29
```

Open other five terminals and executed in sequence

```
cd ~/catkin_ws
source devel/setup.bash
roscore

cd ~/catkin_ws
source devel/setup.bash
roslaunch usb_cam usb_cam-test.launch

cd ~/catkin_ws
source devel/setup.bash
rostopic echo /result_ripe

cd ~/catkin_ws
source devel/setup.bash
roslaunch ros_tensorflow ros_tensorflow_detect.launch

cd ~/catkin_ws
source devel/setup.bash
rosrun image_view image_view image:=/result_ripe
```

The target detection results are shown in the picture.

![Image text](image/6_target_detection_result.png)

## License
This is licensed under the BSD 3-Clause-Clear “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.

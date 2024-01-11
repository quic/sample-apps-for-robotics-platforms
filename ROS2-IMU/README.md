# ROS2-IMU
## Overview
This document shows how to run imu-ros2node with ROS2 environments. And shows the IMUD daemon/library sample code for build own IMU node.
- sensor-daemon: the IMUD service to access IMU data from DSP.
- sensor-client: the library for communicating with IMUD.
- imu-ros2node: the sample implementation of IMU node based on ROS2.

**TIP:** All these components are built-in in RB5 image as binary programs. You can run the imu-ros2node with following steps.

## 1 Set up the WLAN

**1.1 Pull WLAN config to PC**

```bash
adb pull /data/misc/wifi/wpa_supplicant.conf
```

**1.2 Change SSID and password with your Router in wpa_supplicant.conf**

example:

```
network={
    ssid="Monkey1"
    key_mgmt=WPA-PSK
    pairwise=TKIP CCMP
    group=TKIP CCMP
    psk="MY_PASSWORD"
}
```

**1.3 Push wpa_supplicant.conf to device**

```
adb push wpa_supplicant.conf /data/misc/wifi/
```

**1.4 Reboot device and wait for device boot up**

```
adb reboot
```

**1.5 Wait for 1mins after device boot up, and you will be found device have been auto connected to your Router**

```
adb shell
ifconfig wlan0
```

You will see the ip address if WLAN connected, like this:

```
wlan0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 10.91.4.85  netmask 255.255.252.0  broadcast 10.91.7.255
        inet6 fe80::dc7a:257b:cf30:3842  prefixlen 64  scopeid 0x20<link>
        ether 00:03:7f:12:1e:b3  txqueuelen 3000  (Ethernet)
        RX packets 19  bytes 2498 (2.4 KB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 37  bytes 3424 (3.4 KB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```

Now the WLAN connected successfully!

## 2 Set up ROS2 environment

The built-in IMU ROS node supports ROS2 Dashing only.

**2.1 Connect RB5 with adb**

```
adb shell
```

**2.2 Install ROS2 via Debian package**
You can reference the official ROS document:
https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Debians.html
Or you can directly use these commands:

```bash
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
```

Wait until all ROS2 packages are installed completely.

**2.2 Verify the ROS2 installation**

```bash
source /opt/ros/dashing/setup.bash
ros2
```

Will show the ros2 help message if installation is complete.

## 3 Run IMU ROS node

**3.1 Init ROS2 environment**

```
source /opt/ros/dashing/setup.bash
```

**2.2 Run IMU ROS node with `ros2 run` command**

The IMU ROS node is provided with RB5 ubuntu image. You could run the node directly.

```
ros2 run imu-ros2node imu-ros2node
```

**2.3 Test the IMU ROS node**

Open another terminal

```
adb shell
```

Init ROS2 environment and test IMU node with

```
source /opt/ros/dashing/setup.bash
ros2 topic echo /imu
```

Pick up RB5 and shake it, you could see the IMU data are changing in the log:

![imu-data](image/imu-data.png)

## License

This is licensed under the BSD 3-clause-Clear “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.

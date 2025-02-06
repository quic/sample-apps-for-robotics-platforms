
![Qualcomm Innovation Center, Inc.](Docs/images/logo-quic-on@h68.png)

  

# Sample applications for robotics platforms

This is a repository of sample applications that can run on the following development kits:

 - Qualcomm® Robotics RB5 dev kit based on the Qualcomm® QRB5165 processor
 - Qualcomm® Robotics RB2 dev kit based on the Qualcomm® QRB4210 processor
 - Qualcomm® Robotics RB1 dev kit based on the Qualcomm® QRB2210 processor 


Before we jump into the sample apps, lets take a brief look at the supported operating systems available for these development kits.

The development kits support two types of Linux based operating system images. First is the “LE” version, second is the “UBUN” version that has an Ubuntu-based Rootfs. Both types of system images are built using the Yocto build system. For the most part both options are designed to have feature parity, but at a high-level Ubuntu-based system images will support an apt-get package manager and on device compilation, whereas the "LE" version does not come with a package manager and apps will need to be built off-target using the SDK toolchain.

  

The system image is a combination of multiple subsystem binaries for the application processor and various DSPs available on the applicable chipsets. Below are some example system images that can be generated using SDK manager from Thundercomm.

  

 - QRB5165.LE.1.0-xxxxxx : System image based on Linux Kernel 4.xx for RB5 dev kit
 - QRB5165.UBUN.1.0-xxxxxx : System image based on Linux Kernel 4.xx and ubuntu based rootfs for RB5 dev kit 
 - QRB5165.UBUN.2.0-xxxxxx : System image based on Linux Kernel 5.xx and ubuntu based rootfs for RB5 dev kit
  
```
Note: QRB5165.UBUN.1.0-xxxxxx system images have been depricated. Please use the QRB5165.UBUN.2.0-xxxxxx version.
```  

Since the underlying system images running on these development kits might be based on different versions of Linux kernel, the sample apps in this repository are also organized to help developers run the right apps on the applicable platform. Below is a high level folder structure for the sample apps.
 

| Platform | Processor | Linux Kernel | Folder |
|--|--|--|--|
| Qualcomm® Robotics RB5 dev kit |QRB5165 | 4.x | /RB5/linux_kernel_4_x |
| Qualcomm® Robotics RB5 dev kit |QRB5165 | 5.x | /RB5/linux_kernel_5_x |
| Qualcomm® Robotics RB1 and RB2 dev kits | QRB4210 / QRB2210 | 5.x | /RB1RB2/ |

  
  
  
Below is an overview on how sample apps are categorized. Each sample application has its own folder within the category.  Please follow the README in the respective folders to build, deploy and test.

  
  
  

## platform-bringup
```
Contains apps to query device info, GPIO examples, Weston-Client example, WiFi onboarding and more...
```
## peripheral-devices
```
Contains apps to use some of the peripheral devices, like Camera-hal3 sample application.
```
## GStreamer-apps
```
Contains a set of Python and C sample applications that show the use of gstreamer plugins on the development platform.
```
## ROS-apps
```
Contains apps that show the use of ROS middleware for building robotics applications.
```
## AI-ML-apps
```
Contains apps that show how to run AI workloads on the development platform. Also contains apps that implement AI for various use cases.
```
## AWS-Samples
```
Contains instructions on how to enable and test various AWS services on the platform.
```

  

## 16. Sensor-collection-system
```
The sensor collection system is an application that utilizes a collection of drivers in order to concurrently collect data from different sensors.
```

## Getting Started

* [Quick start guide](https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide)

* [Hardware Reference Guide](https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/hardware-reference-guide)

* [Software Reference Manual](https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/software-reference-manual)

* [Discussion Forums](https://developer.qualcomm.com/forum/qdn-forums/hardware/qualcomm-robotics-rb5-development-kit/67886)

  

## Contributions

Please read our [Contributions Page](CONTRIBUTING.md) for more information on contributing features or bug fixes. We look forward to your participation!

  

## Team

A community-driven project maintained by Qualcomm Innovation Center, Inc.

  

## License

Sample applications here are licensed under the BSD 3-clause-Clear “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.
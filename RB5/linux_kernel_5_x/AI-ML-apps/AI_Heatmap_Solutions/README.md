# AI Heatmap Solution on QRB5165 using YoloV5

### Introduction
Heatmap is visual representation of persons or any object in a region with different color encodings based on the distance from one another. Heatmap can be used to do analysis such as customers area of interest for a particular product in a store, avoid overcrowd while checkout and suggest empty lanes.
Person detection can be done using YoloV5 model.
In this repository we have implemented various AI usecases that can be implemented using YoloV5 model on Qualcomm® Robotics RB5 platform.

This solution will use IP camera, real-time input stream analysis based on Qualcomm® Robotics RB5 Edge AI device and computer vision systems. Solution provides an end-to-end application that can be quickly integrated into a new build system.

### Prerequisite
1.  Qualcomm® Robotics RB5 Introduction Guide: 
    * https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide
2.  The Qualcomm® Neural Processing SDK provides tools for model conversion(onnx to dlc), model quantization and execution. 
Refer to the installation steps given in the detailed documentation from the SDK.
    * https://developer.qualcomm.com/sites/default/files/docs/snpe/index.html
3.  ADB to access the device using command prompt and push/pull the files from device.
    * https://developer.android.com/studio/command-line/adb

### Hardware Required

1.	An Ubuntu 20.04 PC
2.	Qualcomm® Robotics RB5 Development kit:
   * https://developer.qualcomm.com/qualcomm-robotics-rb5-kit
4.	A USB camera
5.	A display monitor

![image](https://github.qualcomm.com/storage/user/12959/files/2d85cec3-913e-43af-9a06-e539690bb30c)

### Solution Details
The application implemented following vision based object detection solution:

**Heatmap**

This is to detect person and represent with color encoding based on the people density
![hetamap_result](https://github.qualcomm.com/storage/user/30177/files/fafda0d0-a0f6-4443-ab89-c347c9861ada)

## Usecase Details
1. Instore heatmap for product purchase 
2. Suggest empty lane to avoid overcrowd at stores 

### Documentation Details:

#### Design Details
This application supports object detection use case with YoloV5 model using Qualcomm® Neural Processing SDK.

Refer [Design](./DesignDetails.md)  to understand the implementation details and steps to integrate a model into the application.

#### Device Setup

There are prerequisite and some packages need to be installed to run the application. Please refer to [SetupDevice](./SetupDevice.md) to prepare the setup on Qualcomm® Robotics RB5.

#### Model Preperation

Refer to [ModelPreperation](./ModelPreperation.md) for model generation.

#### Deploy and Execute

Refer [Build](./Build.md) to compile and execute the application after setup is done

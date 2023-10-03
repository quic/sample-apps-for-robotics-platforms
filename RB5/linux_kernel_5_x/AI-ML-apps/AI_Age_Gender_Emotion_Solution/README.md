# AI Age, Gender, Emotion prediction Solution 

### Introduction
AI have enabled remarkable progress in age, gender, and emotion detection using face detection. There are various deep learning models which extracts the meaningful features and perform operations like age prediction, gender prediction,emotion detection, face recognition etc. without any human interaction to reduce the time consumed while checkout at stores.

In this repository we have implemented AI use case on Qualcomm® Robotics RB5 platform.

The solutions can be IP camera, real-time input stream analysis based on Qualcomm® Robotics RB5 Edge AI device and computer vision systems. Solution provides an end-to-end application that can be quickly integrated into a new build system.

### Prerequisite
1.  Qualcomm® Robotics RB5 Introduction Guide: 
    * https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide
2.  The Qualcomm® Neural Processing SDK provides tools for model conversion (onnx to dlc), model quantization and execution. 
Refer to the installation steps given in the detailed documentation from the SDK.
    * https://developer.qualcomm.com/sites/default/files/docs/snpe/index.html
3.  ADB to access the device using command prompt and push/pull the files from device.
    * https://developer.android.com/studio/command-line/adb

### Hardware Required

1.	An Ubuntu 20.04 PC
2.	Qualcomm Robotics RB5 Development kit:
   * https://developer.qualcomm.com/qualcomm-robotics-rb5-kit
4.	A USB camera
5.	A display monitor

![image](https://github.qualcomm.com/storage/user/12959/files/2d85cec3-913e-43af-9a06-e539690bb30c)

### Solution Details
The application implemented following vision-based face detection solutions:
1.Age detection
2.Gender detection
3.Emotion detection


![face](https://github.qualcomm.com/storage/user/31436/files/ec5993fb-6637-4bb7-8ed8-b5a0ac18e322)

Video reference: https://www.youtube.com/watch?v=Q5gQmjxBSb0


### Documentation Details:

#### Design Details
This application supports Age prediction with Face detection use cases using SNPE.

Refer [DesignDetails.md](./docs/DesignDetails.md) to understand the implementation details and steps to integrate a model into the application.

#### Device Setup

There are prerequisite and some packages need to be installed to run the application. Please refer [SetupDevice.md](./docs/SetupDevice.md) to prepare the setup.


#### Deploy and execute

After setup is done. The application can be compiled and executed. Follow the instruction given in [Build.md](./docs/Build.md) to execute the application.



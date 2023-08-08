# CE2.0 SI Tracker Solutions on LU.UM.2.0 build

### Introduction
Object Tracking is a combination of detecting object and assigning unique identity. In each consecutive frame the object will maintain its identity. Multi-object tracking (MOT) aims at tracking objects of a single class or multiple classes. Here an object can be like persion, vehicle, luggage etc.


### Prerequisite
1.  Qualcomm® Robotics RB5 Introduction Guide:
    * https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide
2.  The Qualcomm® Neural Processing SDK provides tools for model conversion(onnx to dlc), model quantization and execution. 
Refer to the installation steps given in the detailed documentation from the SDK.
    * https://developer.qualcomm.com/software/qualcomm-neural-processing-sdk
3.  ADB to access the device using command prompt and push/pull the files from device.
    * https://developer.android.com/studio/command-line/adb

### Hardware Required

1.	An Ubuntu 20.04 PC
2.	Qualcomm Robotics RB5 Development kit: https://developer.qualcomm.com/qualcomm-robotics-rb5-kit
3.	A USB camera
4.	A display monitor

![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/5b3079fd-35be-4d82-a997-51f1357e22e9)

### Solution Details
The application implemented People Tracking using ByteTrack model:

**Person tracking**

This solution will track persons in a frame by assigning unique tracker ID

https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/8a2f6988-16ae-4df2-b68c-0f215f645508

Ref: https://github.com/ifzhang/ByteTrack

### Documentation Details:

#### Design Details
This application supports object detection use cases with YoloX model using Qualcomm® Neural Processing SDK.

Refer Design.md to understand the implementation details and steps to integrate a model into the application.
[Design Details](./docs/DesignDetails.md)

#### Device Setup

There are prerequisite and some packages need to be installed to run the application. Please refer Install.md to prepare the setup on development platform.
[Setup Device](./docs/SetupDevice.md)

#### Model Preperation

Need to prepare model for the solution. Refer ModelPreperation.md for model generation.
[Model Preperation](./docs/ModelPreperation.md)

#### Deploy and Execute

After setup is done. The application can be compiled and executed. Follow the instructions given in Build.md to execute the application on development platform.
[Compile and deploy](./docs/Build.md)

# AI Vision Solutions using YoloV5 Model

### Introduction
There are different workplaces where the safety of the employee is at risk due to the nature of work. There could be serious injury can happen to the employee due any uncontrollable reason. There are different AI solutions that can mitigate or reduce the risk of serious injury.
Other than Safety Solutions, There are more object detection use cases like Trash Detection, Damaged Box detection, People counting can be done using YoloV5 model.
In this repository we have implemented various AI usecases that can be implemented using YoloV5 model on Qualcomm® Robotics RB5 development platform.

We will discuss about various object detection solutions that can be done using YoloV5:
1. People Intrusion Detection
2. Helmet Detection
3. Fire Detection
4. Crack Detection at construction site
5. Trash Detection
6. People Counting
7. Damaged Box detection

To reduce and mitigate the serious accidents, we have developed a set of safety solutions in various industries like Warehouse and Construction sites. These solutions will take input from IP camera and perform real-time input stream analysis on Qualcomm® Robotics RB5 development Edge AI device. These solutions provides an end-to-end application that can be quickly integrated into a new build system.

### Prerequisite
1.  Qualcomm® Robotics RB5 Introduction Guide: 
    * https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide
2.  The Qualcomm® Neural Processing SDK provides tools for model conversion(onnx to dlc), model quantization and execution. 
Refer to the installation steps given in the detailed documentation from the SDK.
    * https://developer.qualcomm.com/software/qualcomm-neural-processing-sdk
3.  ADB to access the device using command prompt and push/pull the files from device.
    * https://developer.android.com/studio/command-line/adb

### Hardware Required

1.	An Ubuntu 18.04 PC
2.	Qualcomm Robotics RB5 Development kit:
   * https://developer.qualcomm.com/qualcomm-robotics-rb5-kit
4.	A USB camera
5.	A display monitor

![image](https://user-images.githubusercontent.com/131336334/237058961-9a0825db-8873-441b-abe2-8a62d5a0f85e.png)

### Solution Details
The application implemented following vision based object detection solutions:

**1. Intrusion Detection**

This is an operation safety issue detection case. If a person enters into restricted areas then AI edge device triggers an event.

**2. Fire Detection**

This is a building safety detection case. This Solution will detect if there is any fire or smoke. 


**3. Helmet Detection**

This solution will perform Helmet Detection for construction and warehouses.

**4. Crack Detection**

This solution will detect if there are any crack in construction site.

**5. Trash Detection**

This solution will seperate the different kind of trash using object detection for recycling.

**6. People Counting**

This solution will count the number of people at a place. 

**7. Damaged Box detection**

Check if the cardboard box is damaged on conveyor belt, this solution is helpful in warehouse.

### Documentation Details:

**1. Design Details**
This application supports object detection use cases with YoloV5 model using SDK.

Refer Design.md to understand the implementation details and steps to integrate a model into the application.
[Design Details](./docs/DesignDetails.md)

**2. Device Setup**

There are prerequisite and some packages need to be installed to run the application. Please refer Install.md to prepare the setup on development kit.
[Setup Device](./docs/SetupDevice.md)

**3. Model Preparation**

Each solution uses different trained YoloV5 Model. Need to prepare model for the solution. Refer ModelPreperation.md for model generation.
[Model Preparation](./docs/ModelPreperation.md)

**4. Deploy and execute**

After setup is done. The application can be compiled and executed. Follow then instruction given in Build.md to execute the application on development platform.
[Compile and Build](./docs/Build.md)

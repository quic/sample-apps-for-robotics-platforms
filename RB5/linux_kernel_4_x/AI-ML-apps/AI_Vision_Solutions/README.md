# AI Vision Solutions using YoloV5 Model

### Introduction
There are different workplaces where the safety of the employee is at risk due to the nature of work. There could be a serious injury that can happen to the employee due to any uncontrollable reason. There are different AI solutions that can mitigate or reduce the risk of serious injury.
Other than Safety Solutions, There are more object detection use cases like Trash Detection, Damaged Box detection, and People counting that can be done using the YoloV5 model.
In this repository, we have implemented various AI use cases that can be implemented using the YoloV5 model on Qualcomm® Robotics RB5 development platform.

We will discuss various object detection solutions that can be done using YoloV5:
1. People Intrusion Detection
2. Helmet Detection
3. Fire Detection
4. Crack Detection at the construction site
5. Trash Detection
6. People Counting
7. Damaged Box detection

To reduce and mitigate serious accidents, we have developed a set of safety solutions in various industries like Warehouses and Construction sites. These solutions will take input from IP camera and perform real-time input stream analysis on Qualcomm® Robotics RB5 development Edge AI device. These solutions provide an end-to-end application that can be quickly integrated into a new build system.

### Prerequisite
1.  Qualcomm® Robotics RB5 Introduction Guide: 
    * https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide
2.  The Qualcomm® Neural Processing SDK provides tools for model conversion(onnx to dlc), model quantization and execution. 
Refer to the installation steps given in the detailed documentation from the SDK.
    * https://developer.qualcomm.com/software/qualcomm-neural-processing-sdk
3.  ADB to access the device using the command prompt and push/pull the files from the device.
    * https://developer.android.com/studio/command-line/adb

### Hardware Required

1.	An Ubuntu 18.04 PC
2.	Qualcomm Robotics RB5 Development kit:
   * https://developer.qualcomm.com/qualcomm-robotics-rb5-kit
4.	A USB camera
5.	A display monitor

![image](https://user-images.githubusercontent.com/131336334/237058961-9a0825db-8873-441b-abe2-8a62d5a0f85e.png)

### Solution Details
The application implemented the following vision-based object detection solutions:

**1. Intrusion Detection**

This is an operational safety issue detection case. If a person enters into restricted areas then AI edge device triggers an event.

![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/b2a395f7-f2c9-4b7f-a671-00bdd78d7936)

**2. Fire Detection**

This is a building safety detection case. This Solution will detect if there is any fire or smoke. 

**3. Helmet Detection**

This solution will perform Helmet Detection for construction and warehouses.

![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/20f9d920-924c-4207-9441-9641322c8412)

**4. Crack Detection**

This solution will detect if there are any cracks in the construction site.

![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/b140330f-12e6-4f63-b83f-88dd5708f086)

**5. Trash Detection**

This solution will separate the different kinds of trash using object detection for recycling.

![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/b97c496c-d66a-46da-a8eb-c20d02ad1f4e)

**6. People Counting**

This solution will count the number of people at a place. 

![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/beaccd8e-67a2-4319-9582-c8f983f70aaf)

**7. Damaged Box detection**

Check if the cardboard box is damaged on the conveyor belt, this solution is helpful in the warehouse.

![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/79d40399-d8a5-47f6-bb0d-d0e9e64a4195)

### Documentation Details:

**1. Design Details**
This application supports object detection use cases with the YoloV5 model using SDK.

Refer Design.md to understand the implementation details and steps to integrate a model into the application.
[Design Details](./docs/DesignDetails.md)

**2. Device Setup**

There are prerequisites and some packages need to be installed to run the application. Please refer Install.md to prepare the setup on the development kit.
[Setup Device](./docs/SetupDevice.md)

**3. Model Preparation**

Each solution uses a different trained YoloV5 Model. Need to prepare the model for the solution. Refer ModelPreperation.md for model generation.
[Model Preparation](./docs/ModelPreperation.md)

**4. Deploy and execute**

After setup is done. The application can be compiled and executed. Follow the instruction given in Build.md to execute the application on the development platform.
[Compile and Build](./docs/Build.md)

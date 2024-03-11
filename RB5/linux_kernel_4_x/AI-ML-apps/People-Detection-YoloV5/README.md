# People Intrusion Detection using qtimlesnpe plugin and Gstreamer framework using YoloV5 Model
This project is designed to use Gstreamer pipeline for People Intrusion Detection on Qualcomm® Robotics RB5 development kit with a USB camera. It will help customer to Identify if a person appears in a restricted area by specifying a detection area, the algorithm can automatically and accurately analyze real-time video and detect if a person enters the area. Solution will reduce the manual monitoring cost.

# Objective

The main objective of this project is to start artificial intelligence (AI) application development for robotics using the Qualcomm Robotics RB5 development kit. This project uses Gstreamer Framework It will walk you through the following steps:

1.	Download the RB5 Ubuntu platform source code

2.	Make modifications in qtimlesnpe plugin for the post processing.

3.	Push the Gstreamer qtimlesnpe plugin libraries to the device.

4.	Run the Gstreamer pipeline using SNPE engine for AI inteferencing on Qualcomm Robotics RB5 development kit.

5. Device need to be connected to active internet connection for installation of packages.

# Prerequisite
1.  RB5 Introduction Guide: 
    * https://github.com/quic/sample-apps-for-robotics-platforms/tree/master/RB5/linux_kernel_4_x/AI-ML-apps/People-Intrusion-Detection/docs/GettingStarted.md
2.  The Qualcomm Neural Processing SDK provides tools for model conversion(onnx to dlc), model quantization and execution. 
Refer the steps given in the detailned documentation in the SDK for installation.
    * https://developer.qualcomm.com/software/qualcomm-neural-processing-sdk
3.  ADB to access the device using command prompt and push/pull the files from device.
    * https://developer.android.com/studio/command-line/adb

# Materials Required / Parts List / Tools

1.	An Ubuntu 18.04 PC
2.	Qualcomm Robotics RB5 Development kit:
   * https://developer.qualcomm.com/qualcomm-robotics-rb5-kit
4.	A USB camera
5.	A display monitor

![image](https://user-images.githubusercontent.com/131336334/235096089-97d1d087-c0ba-43ad-821f-470c8f43beda.png)

# Environment Setup to download Yolov5 Model:

Python Requirements:
* python 3.6.9
* onnx==1.6.0
* onnx-simplifier==0.2.6
* onnxoptimizer==0.2.6
* onnxruntime==1.1.0
* numpy==1.16.5
* protobuf==3.17.3
* torch==1.10.0
* torchvision==0.11.1

```console
sudo apt install python3.6 python3.6-venv build-essential make python3-dev python3.6-dev
```
```console
git clone https://github.com/ultralytics/yolov5.git
```
```console
cd yolov5
```
```console
git checkout v6.0
```
```console
python3.6 -m pip install --upgrade pip
```
```console
python3.6 -m pip install -r requirements.txt
```
```console
python3.6 -m pip install coremltools>=4.1 onnx==1.6.0 scikit-learn==0.19.2 onnxruntime==1.1.0 onnx-simplifier==0.2.6 onnxoptimizer==0.2.6
```

### Export YoloV5m to ONNX:
```console
python3.6 export.py --weights yolov5m.pt --optimize --opset 11 --simplify --include onnx
```

## Setup SNPE
SNPE Setup is needed to convert the onnx model to quantized dlc, please follow the instructions for setting up Neural Processing SDK using the link provided. Please use same version of SNPE throughout the demo.
https://developer.qualcomm.com/docs/snpe/setup.html
```console
export SNPE_ROOT=<path to snpe folder>/snpe-1.68.0.3932
```
Find onnx installation path from pip
```console
python3.6 -m pip show onnx
```
Look for 'Location:' line in output of above command
```console
export ONNX_DIR=<path from Location line>/onnx
```
Setup onnx environment for snpe
```console
cd $SNPE_ROOT
source bin/envsetup.sh -o $ONNX_DIR
```
SNPE currently does not support 5D operator. It requires specify output nodes before 5D Reshape in convert command. The output nodes can be checked in the https://netron.app/.

To check the output layer nodes, Open the model in the Netron app and click on Conv layer.
![image](https://user-images.githubusercontent.com/131336334/235096179-a2985d4f-1002-41e9-8e4c-9e7294551f1c.png)

In attached yolov5m.onnx, the output nodes before 5D is onnx::443 (Conv_271), 496 (Conv_305) and 549 (Conv_339)

![image](https://user-images.githubusercontent.com/131336334/235096209-91c10024-6007-4665-9d4e-6179568dfef9.png)

## This implementation does below functions:
* anchorBoxProcess: 
Get raw data from out nodes before 5D (Conv_271, Conv_305, Conv_339), convert to meaning data (scores, class, bounding boxes).
* doNMS: (non-max suppression): remove overlap boxes
* ShowDetectionOverlay: Overlay detection result at output video/Image

## Convert Onnx Model to DLC
### Convert to DLC
```console
snpe-onnx-to-dlc -i yolov5m.onnx --out_node <output name 1> --out_node <output name 2> --out_node <output name 3>
```
Example corresponding to screenshot above:
```console
snpe-onnx-to-dlc -i yolov5m.onnx --out_node 443 --out_node 496 --out_node 549
```

### Use below command to generate the Quantized model for AIP/DSP. Use the same SNPE SDK version which is installed in device.
Please take reference from inputlist.txt file given in model directory. Create your own inputlist.txt file as per your yolov5m.onnx model. We need to update all the output names in inputlist.txt
```console
snpe-dlc-quantize --input_dlc=yolov5m.dlc --input_list=inputlist.txt --output_dlc=yolov5m_quant.dlc --enable_hta 
```

input.raw file is needed for model quantization. Create a sample input.raw file using below python command.
```console
import numpy as np
((np.random.random((1,3,640,640)).astype(np.float32))).tofile("input.raw")
```

* Please read https://developer.qualcomm.com/sites/default/files/docs/snpe/quantized_models.html to know more about model quantization.

# Source Code / Source Examples / Application Executable:

Below are the resources used for the demo application. It include qtimlesnpe plugin source code, YoloV5 dlc, config file and label file..

## 1.	qtimlesnpe pluing source code:
Please refer the source code for the changes done for YoloV5, Different model may need different post processing steps.
  * https://github.com/quic/sample-apps-for-robotics-platforms/tree/master/RB5/linux_kernel_4_x/AI-ML-apps/People-Intrusion-Detection/src

## 2.	Demo dlc file used in this demo
Download the YoloV5 model from the Yolo official repository. Convert the model to dlc using snpe-onnx-to-dlc and quantize it using snpe-dlc-quantize.

## 3.	Demo config file
Config file is used to provide model information to qtimlesnpe plugin. Make changes in the config file to any changes in execution.
Update the model layer information, label and dlc path in the configuration file.
   * https://github.com/quic/sample-apps-for-robotics-platforms/tree/master/RB5/linux_kernel_4_x/AI-ML-apps/People-Intrusion-Detection/model/mle_snpeyolov5m_quant.config

![image](https://user-images.githubusercontent.com/131336334/235096273-1b44f81b-b790-488f-a40b-75fbbc10f703.png)


#### To define the camera FOV, there is need to set restricted area. It can be set by change the x,y, width and height. These dimensions depends on the camera resolution so it need to be set accordingly.
* x_axis = 100
* y_axis = 100
* width = 400
* height = 400
If restricted area not configured, model will perform detection on complete framebuffer. 

## 4. Label file for YoloV5 model
YoloV5 was trained on coco dataset, it uses coco labels.
* https://github.com/quic/sample-apps-for-robotics-platforms/tree/master/RB5/linux_kernel_4_x/AI-ML-apps/People-Intrusion-Detection/model/coco_labels.txt
   
# Build / Assembly Instructions:

1. Hardware set up:

a.  Connect the Qualcomm Robotics RB5 development kit to the monitor through HDMI cable.
b.  Plugin a keyboard and a mouse to the development board.
c.  Connect the USB camera module to the development board.

# Gstreamer Pipeline
![image](https://user-images.githubusercontent.com/131336334/235096309-3524b94b-f6bd-4a5c-ae47-eff97c8c002a.png)

# Detailed flow diagram
![image](https://user-images.githubusercontent.com/131336334/235096337-64bcd074-e064-4d1f-a138-ed38285e5496.png)

# Steps to Run Demo RB5 Board:

## Push required Demo files to device:

1. Prepare Directory to run the model
```console
adb shell "mkdir /data/misc/camera"
```

2. Push model, config and label file to the device:
```console
adb push yolov5m_quant.dlc /data/misc/camera
```
```console
adb push mle_snpeyolov5m_quant.config /data/misc/camera
```
```console
adb push coco_labels.txt /data/misc/camera
```

3. Compile and Install qtimlesnpe libraries on the target:

   Please refer the steps given in the below document to download, compile and install qtimlesnpe gstreamer plugin libraries.
   * https://github.com/quic/sample-apps-for-robotics-platforms/tree/master/RB5/linux_kernel_4_x/AI-ML-apps/People-Intrusion-Detection/docs/Install.md

# Run the model
## Do step 4-7 to display the live stream on laptop

4. Run adb forward command in command prompt
```console
adb forward tcp:8900 tcp:8900
```

5. Run rtsp server
```console
adb shell
```
```console
gst-rtsp-server -p 8900 -m /live "( udpsrc name=pay0 port=8554 caps=\"application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96\" )"
```

6. Run Gstreamer Pipeline from different shell
```console
adb shell
```
```console
gst-launch-1.0 qtiqmmfsrc ! video/x-raw\(memory:GBM\), format=NV12, width=1280, height=720, framerate=30/1 ! queue ! qtimlesnpe config=/data/misc/camera/mle_snpeyolov5m_quant.config postprocessing=yolov5detection ! queue ! qtioverlay bbox-color=0xFF0000FF ! queue ! omxh264enc target-bitrate=6000000 periodicity-idr=1 interval-intraframes=29 control-rate=max-bitrate ! queue ! h264parse config-interval=-1 ! rtph264pay pt=96 ! udpsink host=127.0.0.1 port=8554
```

7. Connect the device to laptop via USB. Install VLC player and run “Open Network Stream” with rtsp://127.0.0.1:8900/live

## Do step 8-9  to display the live stream on external monitor using hdmi cable

8. Connect the device with the monitor using hdmi cable. Reboot the device.

9. Steps to Run the Pipeline
```console
adb shell
```
```console
export XDG_RUNTIME_DIR="/usr/bin/weston_socket"
```
```console
mkdir -p $XDG_RUNTIME_DIR
```
```console
chmod 0700 $XDG_RUNTIME_DIR
```
```console
/usr/bin/weston --tty=1 --connector=29 &
```
```console
gst-launch-1.0 qtiqmmfsrc ! video/x-raw\(memory:GBM\), format=NV12, width=1280, height=720, framerate=30/1 ! queue ! qtimlesnpe config=/data/misc/camera/mle_snpeyolov5m_quant.config postprocessing=yolov5detection ! queue ! qtioverlay bbox-color=0xFF0000FF ! waylandsink  width=1920 height=1080 async=true sync=false enable-last-sample=false
```
Add GST_DEBUG=qtiml*:5 in gstreamer launch command if you want to enable debug logs for qtimlesnpe plugin.

### Demo 
![image](https://user-images.githubusercontent.com/131336334/235096429-856c3ce6-e6ec-446b-b768-05ac7459b0b1.png)

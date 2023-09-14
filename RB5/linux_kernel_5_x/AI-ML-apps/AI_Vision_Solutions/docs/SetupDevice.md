# How to setup the environment

# 1 Prerequisite

- One x86 Host Workstation. Operating System Ubuntu 20.04， Python 3.6.9
- Qualcomm Robotics RB5 Development kit: https://developer.qualcomm.com/qualcomm-robotics-rb5-kit

# 2 x86 Host Installation and Deep Learning Model Compile
The model used in the application needs to be trained with the dataset. Then convert the model to dlc to run in the application.

## 2.1 Qualcomm® Neural Processing SDK Installation
Download Software Development Kit (SDK) from https://developer.qualcomm.com/software/qualcomm-neural-processing-sdk/tools-archive

The Qualcomm Neural Processing SDK provides tools for model conversion (onnx to dlc), model quantization and execution. Refer the steps given in the detailed documentation in the SDK for installation.
https://developer.qualcomm.com/sites/default/files/docs/snpe/index.html

## 2.2 Prepare Yolov5 AI model

Follow the steps in [ModelPreperation.md](./ModelPreperation.md) to export and compile Yolov5s models.  

Download the people detection pre-trained YoloV5 model
https://github.com/ultralytics/yolov5/releases/download/v6.0/yolov5s.pt

Download Yolov5 Fire and Smoke AI Model 
Refer to Github   https://github.com/gengyanlei/fire-smoke-detect-yolov4/blob/master/yolov5/best.pt

For Helmet Detection and Crack Detection, the model need to be trained on the dataset.

Helmet detection dataset can be download from Kaggle.
https://www.kaggle.com/code/litaldavar/hard-head-detection-with-yolov5/input

## 2.3 Export *.pt file to ONNX file

Recommend install Python version 3.6.9.
Download Yolov5 v6.0 source code

```console
wget https://github.com/ultralytics/yolov5/archive/refs/tags/v6.0.tar.gz
 
tar -zxvf v6.0.tar.gz
cd yolov5-6.0
wget https://github.com/ultralytics/yolov5/releases/download/v6.0/yolov5s.pt
wget https://github.com/gengyanlei/fire-smoke-detect-yolov4/blob/master/yolov5/best.pt
```

Python Requirements:
* python 3.6.9
* protobuf==3.17.3
* onnx==1.9.0
* onnx-simplifier==0.2.6
* onnxoptimizer==0.2.6
* onnxruntime==1.1.0
* numpy==1.16.5
* torch==1.10.0
* torchvision==0.11.1

```console
sudo apt install python3.6 python3.6-venv build-essential make python3-dev python3.6-dev protobuf-compiler libprotobuf-dev
```
```console
python3.6 -m pip install --upgrade pip
```
```console
python3.6 -m pip install -r requirements.txt
```
```console
python3.6 -m pip install coremltools>=4.1 onnx==1.9.0 scikit-learn==0.19.2 onnxruntime==1.1.0 onnx-simplifier==0.2.6 onnxoptimizer==0.2.6
```

### Export YoloV5m to ONNX:
```console
python3.6 export.py --weights yolov5m.pt --optimize --opset 11 --simplify --include onnx -imgsz [416,416]
```

# 3. Convert Onnx Model to DLC 
## 3.1 Setup Qualcomm® Neural Processing SDK
SDK Setup is needed to convert the onnx model to quantized dlc, please follow the instructions for setting up Neural Processing SDK using the link provided. Please use same version of SDK throughout the demo.
https://developer.qualcomm.com/docs/snpe/setup.html
```console
export SNPE_ROOT=<path to snpe folder>/snpe-1.68.0.3932
export ANDROID_NDK_ROOT=<path to android ndk folder>
```
Find onnx installation path from pip
```console
python3.6 -m pip show onnx
```
Look for 'Location:' line in output of above command
```console
export ONNX_DIR=<path from Location line>/onnx
```
Setup onnx environment for SDK
```console
cd $SNPE_ROOT
source bin/envsetup.sh -o $ONNX_DIR
```
SDK currently does not support 5D operator. It requires specify output nodes before 5D Reshape in convert command. The output nodes can be checked in the https://netron.app/.

To check the output layer nodes, Open the model in the Netron app and click on Conv layer.
![image](https://user-images.githubusercontent.com/131336334/237062243-23acd845-fb3e-44a9-9f63-156843e88d7f.png)

In attached yolov5m.onnx, the output nodes before 5D is onnx::443 (Conv_271), 496 (Conv_305) and 549 (Conv_339)

![image](https://user-images.githubusercontent.com/131336334/237062337-77518985-89b3-4bf9-b225-120000732b63.png)

## This implementation does below functions:
* anchorBoxProcess: 
Get raw data from out nodes before 5D (Conv_271, Conv_305, Conv_339), convert to meaning data (scores, class, bounding boxes).
* doNMS: (non-max suppression): remove overlap boxes
* ShowDetectionOverlay: Overlay detection result at output video/Image

## 3.2 Convert to DLC
```console
snpe-onnx-to-dlc -i yolov5m.onnx --out_node <output name 1> --out_node <output name 2> --out_node <output name 3>
```
Example corresponding to screenshot above:
```console
snpe-onnx-to-dlc -i yolov5m.onnx --out_node 443 --out_node 496 --out_node 549
```

## 3.3 Generate the Quantized model for AIP/DSP
Use the same SDK version which is installed in device.
Please take reference from inputlist.txt file given in model directory. Create your own inputlist.txt file as per your yolov5m.onnx model. We need to update all the output names in inputlist.txt
```console
snpe-dlc-quantize --input_dlc=yolov5m.dlc --input_list=inputlist.txt --output_dlc=yolov5m_quant.dlc --enable_hta --use_enhanced_quantizer --optimizations cle --axis_quant
```

input.raw file is needed for model quantization. Create a sample input.raw file using below python command.
```console
import numpy as np
((np.random.random((1,3,640,640)).astype(np.float32))).tofile("input.raw")
```

* Please read https://developer.qualcomm.com/sites/default/files/docs/snpe/quantized_models.html to know more about model quantization.


# 4. X86 Host RTSP Stream Installation
In this section, show how to prepare test video and setup an live555 RTSP streaming server to simulate an IP camera. The installation steps run in the X86 host.

## 4.1 Prepare fire and smoke test video. 
Prepare a test video, if test video is mp4 ,mkv or other formats. Need to convert to H264 raw video.
The following steps demonstrate how to convert MP4 to H264 raw video.

```console

wget https://test-videos.co.uk/vids/bigbuckbunny/mp4/h264/1080/Big_Buck_Bunny_1080_10s_1MB.mp4

sudo apt install ffmpeg

ffmpeg -i Big_Buck_Bunny_1080_10s_1MB.mp4 -f h264 -vcodec libx264 Big_Buck_Bunny_1080_10s_1MB.264
```

## 4.2 Live555 server installation. 

```console
wget http://www.live555.com/liveMedia/public/live555-latest.tar.gz
tar -zxvf live555-latest.tar.gz
cd live/
./genMakefiles linux-64bit
make -j4
cd ..
```

copy test video to mediaServer folder

```console
cp Big_Buck_Bunny_1080_10s_1MB.264 ./live/mediaServer

cd ./live/mediaServer

./live555MediaServer
```
rtsp://192.168.4.111:8554/<filename> is the rtsp url. 
 - "192.168.4.111" is RTSP server IP address
 - 8554 is the default port
 - <filename> is the video file name under mediaServer folder
 In this case, url "rtsp://192.168.4.111:8554/Big_Buck_Bunny_1080_10s_1MB.264" is the video address. 

 ## 4.3 Verify live555 server
 Download and install VLC media player on a Windows desktop  https://www.videolan.org/

Launch VLC player, choose "Media->Open Network Stream"
input RTSP url rtsp://192.168.4.111:8554/Big_Buck_Bunny_1080_10s_1MB.264

![image](https://user-images.githubusercontent.com/131336334/237062106-dbff8cb8-087d-4eff-8747-d096189ece63.png)

 Click play to test if live555 server work.<mark>Note: Make sure the network address is reachable </mark>

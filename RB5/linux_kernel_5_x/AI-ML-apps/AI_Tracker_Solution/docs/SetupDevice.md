# How to setup the environment

# 1 Prerequisite

- One x86 Host Workstation. Operating System Ubuntu 20.04， Python 3.6.9
- Qualcomm Robotics RB5 Development kit: https://developer.qualcomm.com/qualcomm-robotics-rb5-kit

# 2 x86 Host Installation and Deep Learning Model Compile
The model used in the application need to be trained with the dataset. Then convert the model to dlc to run in the application.

## 2.1 Qualcomm® Neural Processing SDK Installation
Download SDK from https://developer.qualcomm.com/software/qualcomm-neural-processing-sdk/tools-archive.

The Qualcomm Neural Processing SDK provides tools for model conversion(onnx to dlc), model quantization and execution. Refer the steps given in the detailed documentation in the SDK for installation.
https://developer.qualcomm.com/sites/default/files/docs/snpe/index.html

### Setup  Qualcomm® Neural Processing SDK
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

## 2.2 Prepare Bytetrack model

Follow the steps in [ModelPreperation.md](./ModelPreperation.md) to export and compile model.  

Download pre-trained bytetrack_s_mot17.pth.tar

https://drive.google.com/file/d/1uSmhXzyV1Zvb4TJJCzpsZOIcw7CCJLxj/view?usp=sharing


## 2.3 Export *.pth file to ONNX file

Recommend install Python version 3.6.9.
```console
sudo apt install python3.6 python3.6-venv build-essential make python3-dev python3.6-dev protobuf-compiler libprotobuf-dev
```
Download ByteTrack source code

```console
cd /home/
https://github.com/ifzhang/ByteTrack.git 
cd ByteTrack
pip3 install -r requirements.txt
python3 setup.py develop
```

```console
pip3 install cython
pip3 install 'git+https://github.com/cocodataset/cocoapi.git#subdirectory=PythonAPI'
pip3 install cython_bbox
```

### Export to ONNX:
```console
python3 tools/export_onnx.py --output-name bytetrack_s.onnx -f exps/example/mot/yolox_s_mix_det.py -c pretrained/bytetrack_s_mot17.pth.tar
```

To check the output-layers and output-tensors nodes, Open the model in the Netron app and click on output layer as mentioned in the image
![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/c7c916ff-e4a4-403f-ae8b-d3b28800fd7a)


## Convert Onnx Model to DLC
### Convert to DLC
```console
snpe-onnx-to-dlc -i bytetrack_s.onnx --out_node <output name 1> --out_node <output name 2> --out_node <output name 3>
```
Example corresponding to screenshot above:
```console
snpe-onnx-to-dlc -i bytetrack_s.onnx --out_node output
```

### Use below command to generate the Quantized model for AIP/DSP. Use the same SNPE SDK version which is installed in device.
Please take reference from inputlist.txt file given in model directory. Create your own inputlist.txt file as per your bytetrack_s.onnx model. We need to update all the output names in inputlist.txt
```console
snpe-dlc-quantize --input_dlc=bytetrack_s.dlc --input_list=inputlist.txt --output_dlc=bytetrack_s_quant.dlc
```

input.raw file is needed for model quantization. Create a sample input.raw file using below python command.
```console
import numpy as np
((np.random.random((1,3,608,1088)).astype(np.float32))).tofile("input.raw")
```

* Please read https://developer.qualcomm.com/sites/default/files/docs/snpe/quantized_models.html to know more about model quantization.


# 3. X86 Host RTSP Stream Installation
In this section, show how to prepare test video and setup an live555 RTSP streaming server to simulate an IP camera. The installation steps run in the X86 host.

## 3.1 Prepare people video. 
Prepare a test video, if test video is mp4 ,mkv or other formats. Need to convert to H264 raw video.
The following steps demonstrate how to convert MP4 to H264 raw video.

```console

sudo apt install ffmpeg

ffmpeg -i /home/ByteTrack/videos/palace.mp4 -f h264 -vcodec /home/ByteTrack/videos/palace.264
```

## 3.2 Live555 server installation. 

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
cp /home/ByteTrack/videos/palace.264 ./live/mediaServer
cd ./live/mediaServer
./live555MediaServer
```

 rtsp://192.168.4.111:8554/<filename> is the rtsp url. 
 - "192.168.4.111" is RTSP server IP address
 - 8554 is the default port
 - <filename> is the video file name under mediaServer folder
 In this case, url "rtsp://192.168.4.111:8554/Big_Buck_Bunny_1080_10s_1MB.264" is the video address. 

 ## 3.3 Verify live555 server
 Download and install VLC media player on a Windows desktop  https://www.videolan.org/

Launch VLC player, choose "Media->Open Network Stream"
input RTSP url rtsp://192.168.4.111:8554/Big_Buck_Bunny_1080_10s_1MB.264

![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/6111f504-bdf1-41c8-9c3d-e094fb4b9135)

 Click play to test if live555 server work.<mark>Note: Make sure the network address is reachable </mark>

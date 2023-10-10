# How to setup the environment

# 1 Prerequisite

- One x86 Host Workstation. Operating System Ubuntu 20.04， Python 3.6.9
- Qualcomm Robotics RB5 Development kit: https://developer.qualcomm.com/qualcomm-robotics-rb5-kit

# 2 x86 Host Installation and Deep Learning Model Compile
The model used in the application needs to be trained with the dataset. Then convert the model to dlc to run in the application.

## 2.1  Qualcomm® Neural Processing Software Development Kit (SDK) Installation
Download Software Development Kit (SDK) from https://developer.qualcomm.com/software/qualcomm-neural-processing-sdk/tools-archive

The Qualcomm® Neural Processing Software Development Kit (SDK) provides tools for model conversion(onnx to dlc/caffe to dlc), model quantization and execution. Refer the steps given in the detailed documentation in the SDK for installation.
https://developer.qualcomm.com/sites/default/files/docs/snpe/index.html

## 2.2 Prepare AI model

Download centerface.onnx model from repo
https://github.com/Star-Clouds/CenterFace.git

Download age_net.caffemodel ,age_deploy.prototxt from repo https://github.com/Pawankhatri2k13/Gender-and-Age-Detection/tree/master.
 
Dowload gender_googlenet.onnx model from repo https://github.com/onnx/models.git

Download FER_trained_model.pt from GitHub - shangeth/Facial-Emotion-Recognition-PyTorch-ONNX and convert it to ONNX(emotion.onnx)


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


# 3. Convert Onnx , Caffe Models to DLC 
## 3.1 Setup SNPE
SNPE Setup is needed to convert the onnx, caffe models to quantized dlc, please follow the instructions for setting up Qualcomm® Neural Processing Software Development Kit (SDK) using the link provided. Please use same version of SNPE throughout the demo.
https://developer.qualcomm.com/docs/snpe/setup.html
```console
export SNPE_ROOT=<path to snpe folder>/snpe-1.68.0.3932
export ANDROID_NDK_ROOT=<path to android ndk folder>
```
Find onnx, caffe installation path from pip
```console
python3.6 -m pip show onnx
```
```console
python3.6 -m pip show caffe
```
Look for 'Location:' line in output of above command.
```console
export ONNX_DIR=<path from Location line>/onnx
```
```console
export CAFFE_DIR=<path from Location line>/caffe
```
Setup onnx environment for snpe
```console
cd $SNPE_ROOT
source bin/envsetup.sh -o $ONNX_DIR
```
Setup caffe environment for snpe
```console
cd $SNPE_ROOT
source bin/envsetup.sh -o $CAFFE_DIR
```
The output nodes can be checked in the https://netron.app/.

### Centerface

![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/a7c4eda6-75b9-4ca0-9cae-532078c3ead0)

In attached snapshot, the output nodes 536 (Conv2d_39), 538 (Conv2d_40), 539 (Conv2d_41) and 540 (Conv2d_42)
![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/a43410e1-3464-4324-b95f-e90bdb0e076b)

### Age
![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/fdd249cb-731d-490d-9db1-140bb07558a4)

In attached snapshot, the output node is prob.

### Gender
![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/4837b8be-7afe-4b0a-93bf-dc37c2bb7602)

In attached snapshot, the output nodes is loss3/loss3_Y(loss3/loss3)

![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/f777e840-6782-46fb-9a85-3f41e6c21e3e)


### Emotion
![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/0102c6be-5387-4401-a493-2867cc6fa163)

In attached snapshot, the output node is 95

![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/96cd35b9-f64d-44a5-bf38-f34faecb94fc)

## This implementation does below functions:
Get raw data from out nodes 536 (Conv2d_39), 538 (Conv2d_40), 539 (Conv2d_41) and 540 (Conv2d_42), convert to meaning data (bounding boxes) which is further processed to provide age at prob, gender at loss3/loss3_Y and emotion at 94.

## 3.2 Convert to DLC
```console
snpe-onnx-to-dlc -i centerface.onnx --out_node 536 --out_node 538 --out_node 539 --out_node 540
snpe-caffe-to-dlc --input_network age_deploy.prototxt --caffe_bin age_net.caffemodel  
snpe-onnx-to-dlc -i gender_googlenet.onnx
snpe-onnx-to-dlc -i emotion.onnx --out_node 94

```

## 3.3 Generate the Quantized model for AIP/DSP
Use the same SNPE SDK version which is installed in device.
Please take reference from inputlist.txt file given in model directory. Create your own inputlist.txt file as per your yolov5m.onnx model. We need to update all the output names in inputlist.txt.

### Centerface
```console
snpe-dlc-quantize --input_dlc=centerface.dlc --input_list=inputlist.txt --output_dlc=centerface_quant.dlc --enable_hta 
```

input.raw file is needed for model quantization. Create a sample input.raw file using below python command.
```console
import numpy as np
((np.random.random((1,3,640,640)).astype(np.float32))).tofile("input.raw")
```

### Age
```console
snpe-dlc-quantize --input_dlc=age_caffe.dlc --input_list=inputlist.txt --output_dlc=age_caffe_quant.dlc --enable_hta 
```

input.raw file is needed for model quantization. Create a sample input.raw file using below python command.
```console
import numpy as np
((np.random.random((1,3,227,227)).astype(np.float32))).tofile("input.raw")
```

### Gender
```console
snpe-dlc-quantize --input_dlc=gender_googlenet.dlc --input_list=inputlist.txt --output_dlc=gender_google_quant.dlc --enable_hta 
```

input.raw file is needed for model quantization. Create a sample input.raw file using below python command.
```console
import numpy as np
((np.random.random((1,3,224,224)).astype(np.float32))).tofile("input.raw")
```

### Emotion
```console
snpe-dlc-quantize --input_dlc=Emotion1.dlc --input_list=inputlist.txt --output_dlc=emotion_quant.dlc --enable_hta 
```

input.raw file is needed for model quantization. Create a sample input.raw file using below python command.
```console
import numpy as np
((np.random.random((1,3,48,48)).astype(np.float32))).tofile("input.raw")
```


* Please read https://developer.qualcomm.com/sites/default/files/docs/snpe/quantized_models.html to know more about model quantization.


# 4. X86 Host RTSP Stream Installation
In this section, show how to prepare test video and setup an live555 RTSP streaming server to simulate an IP camera. The installation steps run in the X86 host.

## 4.1 Prepare test video. 
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

![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/5e9c1410-cb85-4a04-9a3a-b3211f484cea)

 Click play to test if live555 server work.<mark>Note: Make sure the network address is reachable </mark>

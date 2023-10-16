# Model Preparation

## 1. Qualcomm® Neural Processing Software Development Kit Setup

Qualcomm® Neural Processing Software Development Kit (SDK) is required to convert the onnx, caffe models to quantized dlc, please follow the instructions for setting up using the link provided. Please use same version of SNPE throughout the demo.
https://developer.qualcomm.com/docs/snpe/setup.html
### Prerequisite

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
### Environment Setup

```console
export SNPE_ROOT=<path to snpe folder>/snpe-1.68.0.3932
export ANDROID_NDK_ROOT=<path to android ndk folder>
```
Find onnx,caffe installation path from pip
```console
python3.6 -m pip show onnx
```
```console
python3.6 -m pip show caffe
```
Look for 'Location:' line in output of above command
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

# 2. Convert Onnx , Caffe Models to DLC 
## 2.1 To check Output nodes

The output nodes can be checked in the https://netron.app/.

### Centerface
Download centerface.onnx model from repo https://github.com/Star-Clouds/CenterFace.git
![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/0f39864b-1451-4905-986a-f43b1ce5be76)

In attached snapshot, the output nodes 536 (Conv2d_39), 538 (Conv2d_40), 539 (Conv2d_41) and 540 (Conv2d_42)
![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/0c848182-e167-4b80-8320-f3e3f0bc5d0f)

### Age
Download age_net.caffemodel ,age_deploy.prototxt from repo GitHub - https://github.com/Pawankhatri2k13/Gender-and-Age-Detection.git
![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/16cd9eec-89d6-4c07-a966-da5bdd8ab961)

In attached snapshot, the output node is prob.

### Gender
Dowload gender_googlenet.onnx model from repo https://github.com/onnx/models.git
![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/c28a5d3f-9e22-4b79-b4a3-29edea1909a2)

In attached snapshot, the output nodes is loss3/loss3_Y(loss3/loss3)
![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/c62f2f88-ba08-4458-b3a1-2c4b730821fb)


### Emotion
Download FER_trained_model.pt from GitHub - https://github.com/shangeth/Facial-Emotion-Recognition-PyTorch-ONNX.git and convert it to ONNX(emotion.onnx)
![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/f29938fd-df74-4c95-86bc-4404cacf2775)

In attached snapshot, the output node is 95
![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/0d26a13e-1d47-49e0-8e32-9e418b57da84)


## This implementation does below functions:
Get raw data from out nodes 536 (Conv2d_39), 538 (Conv2d_40), 539 (Conv2d_41) and 540 (Conv2d_42), convert to meaning data (bounding boxes) which is further processed to provide age at prob, gender at loss3/loss3_Y and emotion at 94.

## 2.2 Convert to DLC
```console
snpe-onnx-to-dlc -i centerface.onnx --out_node 536 --out_node 538 --out_node 539 --out_node 540
snpe-caffe-to-dlc --input_network age_deploy.prototxt --caffe_bin age_net.caffemodel  
snpe-onnx-to-dlc -i gender_googlenet.onnx
snpe-onnx-to-dlc -i emotion.onnx --out_node 94

```

## 2.3 Generate the Quantized model for DSP
Use the same SNPE SDK version which is installed in device.
Please take reference from inputlist.txt file given in model directory. Create your own inputlist.txt file as per your yolov5m.onnx model. We need to update all the output names in inputlist.txt

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

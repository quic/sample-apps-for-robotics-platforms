# Model Preparation

## 1. Prerequisite

Follow SetupDevice.md for Setup preperation on device before this document.

Bytetrack model is for object tracking. Object tracking models are trained on the set of images and search for the object classes. When the particular object is found, this model will return the coordinate of the bounding box, probability, unique ID and their class. Pretrained Bytetrack Object tracking model is Trained on CrowdHuman, MOT17, Cityperson and ETHZ, evaluate on MOT17 train. Here we will discuss on Object tracking usecases using Bytetrack.

The pre-trained Bytetrack model is trained for people detection. To use the same model for other object-tracking use cases then the model needs to be trained again. 
Need to execute following steps for every solution to generate any Bytetrack model for SNPE execution:

a) Download MOT17 and CrowdHuman or required object tracking data

b) Write Bytetracking Training configuration

c) Run Bytetrack training

d) Evaluate Bytetrack performance

e) Visualize Bytetrack training data

f) Export Bytetrack model to onnx format

g) Convert the model from onxx to dlc format using SNPE tools

h) Quantize the model using SNPE tools

## 2. ByteTrack environment setup and model conversion

### 2.1 Clone ByteTrack repository and install dependencies
##### ByteTrack by ifzhang, MIT license
```console
cd /home/
git clone https://github.com/ifzhang/ByteTrack.git
cd ByteTrack
```

Python Requirements:
* python 3.8.0
* protobuf==4.22.0
* onnx==1.8.1
* onnx-simplifier==0.3.5
* onnxoptimizer==0.3.13
* onnxruntime==1.8.0
* numpy==1.23.1
* torch==1.12.0
* torchvision==0.13.0

```console
sudo apt install python3.6 python3.6-venv build-essential make python3-dev python3.6-dev protobuf-compiler libprotobuf-dev
```
```console
python3.6 -m pip install --upgrade pip
```
```console
python3.6 -m pip install -r requirements.txt
python3 setup.py develop
pip3 install cython
pip3 install 'git+https://github.com/cocodataset/cocoapi.git#subdirectory=PythonAPI'
pip3 install cython_bbox
```

If there are any issues in installing cython_bbox, it can be installed from tar package. Download the tar package from PyPI, extracted it and edit the source files.
cd into the package directory and then change line 31 in the setup.py file from extra_compile_args=['-Wno-cpp'], to extra_compile_args = {'gcc': ['/Qstd=c99']}.

Then install this package from the local source using "pip install -e /path/cython_bbox-0.1.3.tar/dist/cython_bbox-0.1.3/cython_bbox-0.1.3".

### 2.2 Downloading pretrained model and convert to ONNX:
#### 2.2.1 Dowload pretrained model from the below link:

These steps will download the pretrained Bytetrack model. Default model can do People Tracking. Model need to be trained again with the correct dataset, if you need to track different object class.
https://drive.google.com/file/d/1uSmhXzyV1Zvb4TJJCzpsZOIcw7CCJLxj/view?usp=sharing


#### 2.2.2 Export to ONNX:
```console
cd /home/ByteTrack
python3 tools/export_onnx.py --output-name bytetrack_s.onnx -f exps/example/mot/yolox_s_mix_det.py -c pretrained/bytetrack_s_mot17.pth.tar
```

If you want to evaluate the pretrained model, Please go to [Setup Device](./SetupDevice.md) for model conversion and quantization.

## 3. Train and evaluate Bytetrack model on person tracking dataset
Steps mentioned below should followed to train the YoloX model.
#### Step 1: Download Pretrained YOLOX model
Default yolox_x.pth pretrained model is trained on coco dataset. 
```console
wget https://github.com/Megvii-BaseDetection/storage/releases/download/0.0.1/yolox_x.pth
```
Refer SetupDevice.md for Model conversion and Quantization.
Refer Build.md for Model integration and Run the application.

#### Step 2: Download person datasets

Download the crowdhuman and mot17 dataset from:

 https://www.crowdhuman.org/download.html and https://motchallenge.net/data/MOT17/ 

Unzip the repository in ByteTrack/datasets model directory.

#### Step 3: Organize the dataset
Updtate Dataset organization as shown below 

datasets/

   |——————mot
   
          └——————train        
          └——————test
   
   └——————crowdhuman
   
          └——————Crowdhuman_train
          └——————Crowdhuman_val
   
          └——————annotation_train.odgt  
          └——————annotation_val.odgt
annotation_train.odgt and annotation_val.odgt files contain the dataset details. 

#### Step 4: Convert dataset to COCO format and mix different training datasets
```console
cd /home/ByteTrack
python3 tools/convert_mot17_to_coco.py
python3 tools/convert_crowdhuman_to_coco.py
python3 tools/mix_data_ablation.py
```

#### Step 5: Train People Detection on YOLOX
```console
Input Arguments:

* DEVICES: Number of CUDA devices
* BATCH_SIZE: determine batch size
* fp16: Adopting mix precision training
```
```console
cd /home/ByteTrack
python3 tools/train.py -f exps/example/mot/yolox_x_ablation.py -d 8 -b 48 --fp16 -o -c pretrained/yolox_x.pth
```
Wait for training to finish. Once it is done, continue with the following steps.

#### Step 6: Evaluate trained YoloX model
```console
* PATH: path to images or video
* save_result: whether to save the inference result of image/video
* fuse: Fuse conv and bn for testing
* fp16: Adopting mix precision evaluating
```
```console
cd /home/ByteTrack
python3 tools/demo_track.py video -f exps/example/mot/yolox_x_mix_det.py -c pretrained/bytetrack_x_mot17.pth.tar --fp16 --fuse --save_result
```

#### Step 7: Convert the trained model to onnx format.
```console
cd /home/ByteTrack
python3 tools/export_onnx.py --output-name bytetrack_s.onnx -f exps/example/mot/yolox_s_mix_det.py -c pretrained/bytetrack_s_mot17.pth.tar
```
After export command, bytetrack_s.onnx file will be generated that can be used in the application.

Refer SetupDevice.md to Setup SNPE and Model conversion to DLC and Model quantization.
Use Build.md for Model integration, Compilation and Execution.


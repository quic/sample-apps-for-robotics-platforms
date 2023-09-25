# Model Preparation

#### Prerequisite
Follow SetupDevice.md for Setup preperation on device before this document.

YOLOV5 models are object detection models. Object detection models are trained on the set of images and search for the object classes. When the particular object is found, this model will return the coordinate of the bounding box, probability and their class. Pretrained YoloV5 Object detection models is trained and evaluated on the COCO dataset which contains a broad range of 80 classes. We can train the YoloV5 model on top of the pretrained model on different classes with the new training data. Here we will discuss on different Object detection usecases using YOLOv5.


Overall we need to execute following steps for every solution to generate any YoloV5 model using Qualcomm® Neural Processing SDK:

a) Download custom YOLOv5 object detection data

b) Write YOLOv5 Training configuration

c) Run YOLOv5 training

d) Evaluate YOLOv5 performance

e) Visualize YOLOv5 training data

f) Export YoloV5 model to onnx format

g) Convert the model from onxx to dlc format using Qualcomm® Neural Processing SDK

h) Quantize the model using Qualcomm® Neural Processing SDK

## Download YOLOv5 v6.0 Source Code and model

### Download Yolov5 v6.0 pretrained model 

```console
wget https://github.com/ultralytics/yolov5/archive/refs/tags/v6.0.tar.gz
tar -zxvf v6.0.tar.gz
cd yolov5-6.0
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

### Export YoloV5s to ONNX:
```console
python3.6 export.py --weights yolov5s.pt --optimize --opset 11 --simplify --include onnx
```

## Solutions
### YOLOv5 model on custom Helmet,person and head detection dataset

Steps mentioned should followed to train the YoloV5 model for any other dataset as well. Use training dataset corresponding to the required solution. 

#### Step 1

Download the dataset from: https://www.kaggle.com/datasets/andrewmvd/hard-hat-detection in "Yolo V5 Pytorch" format.

Unzip the repository in yolov5-6.0 model directory.

#### Step 2
YoloV5 model configuration: data.yaml file is already present in the custom dataset. If not present, create data.yaml with below content. Update the path correctly as per the dataset location. nc = {number of classes}. It is 3 in this training dataset.

```console
train: ../train/images
val: ../test/images

nc: 3
names: ['helmet', 'person', 'head']
```

Create yaml config file similar to yolov5s.yaml with YoloV5 configuration. Update the label count as per the model output. 


#### Step 3: Train on YOLOv5
Input Arguments:

* img: define input image size
* batch: determine batch size
* epochs: define the number of training epochs. (Note: often, 3000+ are common here!)
* data: set the path to our yaml file
* cfg: specify our model configuration
* weights: specify a custom path to weights. (Note: you can download weights from the Ultralytics Google Drive folder)
* name: result names
* nosave: only save the final checkpoint
* cache: cache images for faster training

```console
cd yolov5-6.0/
python train.py --img 416 --batch 1 --epochs 100 --data '../data.yaml' --cfg ./yolov5s.yaml --weights '' --name helmet_yolov5s  --cache
```
Wait for training to finish. Once it is done, continue with the following steps.

#### Step 4: Evaluate trained YoloV5 model on customer dataset

Please give a valid test video in --source to evaluate the model detection.
```console
cd yolov5-6.0/
python detect.py --weights runs/train/helmet_yolov5s/weights/best.pt --img 416 --conf 0.4 --source ../test.mp4
```

#### Step 5: Convert the trained model to onnx format.
```console
python3.6 export.py --weights runs/train/helmet_yolov5s/weights/best.pt --optimize --opset 11 --simplify --include onnx -imgsz [416,416]
```
After export command, best.onnx file will be generated that can be used in the application.

Refer SetupDevice.md to Setup Qualcomm® Neural Processing SDK and Model conversion to DLC and Model quantization.
Use Build.md for Model integration, Compilation and Execution.

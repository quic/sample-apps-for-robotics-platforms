# Model Preparation

#### Prerequisite
Follow [SetupDevice.md](./SetupDevice.md) for Setup preperation on device before executing this document.

YOLOV5 models are object detection models. Object detection models are trained on the set of images and search for the object classes. When the particular object is found, this model will return the coordinate of the bounding box, probability and their class. Pretrained YoloV5 Object detection models is trained and evaluated on the COCO dataset which contains a broad range of 80 classes. We can train the YoloV5 model on top of the pretrained model on different classes with the new training data. Here we will discuss on different Object detection usecases using YOLOv5.

Overall we need to execute following steps for every model to execute efficiently on Qualcomm® Neural Processing SDK:
1. Download custom YOLOv5 object detection data
2. Write YOLOv5 Training configuration
3. Run YOLOv5 training
4. Evaluate YOLOv5 performance
5. Visualize YOLOv5 training data
6. Export YoloV5 model to onnx format
7. Convert the model from onxx to dlc format using SDK.
8. Quantize the model using SDK.

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

### 1: People Intrusion Detection

Default yolov5s.onnx pretrained model is trained on coco dataset. We can use Class 0 of pretrained model for people intrusion detection. 
Refer [SetupDevice.md](./SetupDevice.md) for Model conversion and Quantization.
Refer [Build.md](./Build.md) for Model integration and Run the application.

### 2: YOLOv5 model on custom Helmet detection dataset

Steps mentioned in Helmet detection should followed to train the YoloV5 model for any other dataset as well. Use training dataset corresponding to the required solution. 

#### Step 1

Download the dataset from: https://public.roboflow.com/object-detection/hard-hat-workers/1/download in "Yolo V5 Pytorch" format.

Unzip the repository in yolov5-6.0 model directory.

#### Step 2
YoloV5 model configuration: data.yaml file is already present in the custom dataset. If not present, create data.yaml with below content. Update the path correctly as per the dataset location. nc = {number of classes}. It is 3 in this training dataset.

```console
train: ../train/images
val: ../test/images

nc: 3
names: ['head', 'helmet', 'person']
```

Create another file yolov5s.yaml with YoloV5 configuration. 

```console
# YOLOv5 🚀 by Ultralytics, GPL-3.0 license

# Parameters
nc: 3  # number of classes
depth_multiple: 0.33  # model depth multiple
width_multiple: 0.50  # layer channel multiple
anchors:
  - [10,13, 16,30, 33,23]  # P3/8
  - [30,61, 62,45, 59,119]  # P4/16
  - [116,90, 156,198, 373,326]  # P5/32

# YOLOv5 v6.0 backbone
backbone:
  # [from, number, module, args]
  [[-1, 1, Conv, [64, 6, 2, 2]],  # 0-P1/2
   [-1, 1, Conv, [128, 3, 2]],  # 1-P2/4
   [-1, 3, C3, [128]],
   [-1, 1, Conv, [256, 3, 2]],  # 3-P3/8
   [-1, 6, C3, [256]],
   [-1, 1, Conv, [512, 3, 2]],  # 5-P4/16
   [-1, 9, C3, [512]],
   [-1, 1, Conv, [1024, 3, 2]],  # 7-P5/32
   [-1, 3, C3, [1024]],
   [-1, 1, SPPF, [1024, 5]],  # 9
  ]

# YOLOv5 v6.0 head
head:
  [[-1, 1, Conv, [512, 1, 1]],
   [-1, 1, nn.Upsample, [None, 2, 'nearest']],
   [[-1, 6], 1, Concat, [1]],  # cat backbone P4
   [-1, 3, C3, [512, False]],  # 13

   [-1, 1, Conv, [256, 1, 1]],
   [-1, 1, nn.Upsample, [None, 2, 'nearest']],
   [[-1, 4], 1, Concat, [1]],  # cat backbone P3
   [-1, 3, C3, [256, False]],  # 17 (P3/8-small)

   [-1, 1, Conv, [256, 3, 2]],
   [[-1, 14], 1, Concat, [1]],  # cat head P4
   [-1, 3, C3, [512, False]],  # 20 (P4/16-medium)

   [-1, 1, Conv, [512, 3, 2]],
   [[-1, 10], 1, Concat, [1]],  # cat head P5
   [-1, 3, C3, [1024, False]],  # 23 (P5/32-large)

   [[17, 20, 23], 1, Detect, [nc, anchors]],  # Detect(P3, P4, P5)
  ]
```

#### Step 3: Train Helmet Detection on YOLOv5
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

#### Step 4: Evaluate trained YoloV5 model on customer Helmet dataset

Please give a valid test video in --source to evaluate the model detection.
```console
cd yolov5-6.0/
python detect.py --weights runs/train/helmet_yolov5s/weights/best.pt --img 416 --conf 0.4 --source ../test.mp4
```

#### Step 5: Convert the trained model to onnx format.
```console
python3.6 export.py --weights runs/train/helmet_yolov5s/weights/best.pt --optimize --opset 11 --simplify --include onnx
```
After export command, best.onnx file will be generated that can be used in the application.

Refer [SetupDevice.md](./SetupDevice.md) to Setup SDK and Model conversion to DLC and Model quantization.
Use [Build.md](./Build.md) for Model integration, Compilation and Execution.

#### 3: YOLOv5 model Fire Detection dataset

For demo purpose, download the pretrained weights for fire detection and smoke detection from below link.
Refer to Github https://github.com/gengyanlei/fire-smoke-detect-yolov4/blob/master/yolov5/best.pt

Refer Step 5 in Section 2 of Solutions to convert the model onnx format.
Refer [SetupDevice.md](./SetupDevice.md) to Setup SDK and Model conversion to DLC and Model quantization.
Use [Build.md](./Build.md) for Model integration, Compilation and Execution.

#### 4: People Counting

Default YoloV5 model can be used for People counting. It is pretrained on coco dataset which has People in Class 0.

#### 5: Damaged Box detection

Dataset images are avaiable at: 
https://bwsyncandshare.kit.edu/s/s5Yr4QQrCEfdigo/download

Labelling of dataset need to be done seperately. Refer below link for labelling. 
https://docs.ultralytics.com/yolov5/train_custom_data/

Follow Solution 2 (Helmet detection) for further procedure.

#### 6: Trash detection

Dataset images are avaiable at: 
https://www.kaggle.com/datasets/mostafaabla/garbage-classification/code?select=garbage_classification

Labelling of dataset need to be done seperately. Refer below link for labelling. 
https://docs.ultralytics.com/yolov5/train_custom_data/

Follow Solution 2 (Helmet detection) for further procedure.

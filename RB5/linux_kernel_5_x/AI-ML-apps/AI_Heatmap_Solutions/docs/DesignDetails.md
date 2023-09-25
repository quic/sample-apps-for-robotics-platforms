# SI Solution Design Specification

#  Overview
Below diagram shows the application architecture. This is a simple Producer – Consumer pattern. 
- The application creates a thread to pull up the Camera/RTSP video stream and uses the VPU hardware to decode it. Each video corresponds to a separate thread.
- After the decoding is completed, video frame enters a thread-safe queue.
- When the application starts, a thread is created which is responsible for AI inference, and this thread keeps polling and pop video frames from queue. After getting the latest frames, SM8250 call OpenCV resize, normalization image, convert to tensor then using SNPE for inference.
- After the inference is completed, the CPU performs post-processing on the inference output results and calls the notification callback function to print the result.

![heatmap_pipeline](https://github.qualcomm.com/storage/user/30177/files/5d3a168d-1a30-4c7e-aa5c-7135c7de0255)


#  Global Configuration

Configuration class is a global variables (./inc/Configuration.h). The setting includes local path of the model, RTSP address and so on. It is implemented with  singleton pattern . The variables were persisted to a file : ./data/config.json

Table 1-1 show all the configuration items:

**Input Configuration**

| key                      | Value  | Description                                    |
| ------------------------ | -----  | ---------------------------------------------- |
| input-config-name        | string | Name of the input config                       |
| stream-type              | string | Input stream type camera or rtsp               |
| stream-width             | int    | Width of the input stream                      |
| stream-height            | int    | Height of the input stream                     |
| SkipFrame                | int    | Number of frames to skip                       |
| camera-url               | string | rtsp stream path if the input stream is rtsp   |

**Model Configuration**

| key                      | Value  | Description                                    |
| ------------------------ | -----  | ---------------------------------------------- |
| model-name               | string | Name of the model                              |
| model-type               | string | Type of the model for ex: yolov5               |
| model-path               | string | Path of the dlc file                           |
| label-path               | string | Path of the label file                         |
| runtime                  | string | SNPE Runtime (GPU, CPU, DSP)                   |
| nms-threshold            | float  | NMS threshold                                  |
| conf-threshold           | float  | Confidence threshold                           |
| labels                   | int    | Number of labels + 5                           |
| input-layers             | string | Name of the input layers                       |
| output-layers            | string | Name of the output layers                      |
| output-tensors           | string | Name of the output tensors                     |


**Solution Configuration**


| key                      | Value  | Description                                    |
| ------------------------ | -----  | ---------------------------------------------- |
| solution-name            | string | Name of the Solution                           |
| model-name               | string | Name of the model configuration to be used     |
| input-config-name        | string | Name of the input configuration to be used     |
| Enable                   | bool   | 1 to Enable and 0 to Disable the solution      |
| output-type              | string | Filesink to save the output in mp4             |
|                          |        | Wayland if display the output on hdmi monitor  |
| output-path              | string | Path of the output, Enabled if output type     |
|                          |        | is filesink                                    |

src/Configuration.cpp

#  RTSP Stream Decode
We use the GStreamer plugin to decode the RTSP video stream and application uses "qtivdec" plugin to decode H264 video frame, this plugin utilizes the SM8250 VPU hardware unit to accelerated decoding. 
The entire GStreamer decoding pipeline as below command:

```console
! rtspsr ! rtph264depay ! h264parse ! qtivdec ! qtivtransform ! ! video/x-raw,format=BGR ! appsink
```

class "StreamDecode"  decode video stream. refer to inc/StreamDecode.h

#  Camera Stream Decode
We use the GStreamer "qtiqmmfsrc" plugin to get the input feed from camera and application uses "qtivtransform" plugin for YUV to RAW conversion.

The entire GStreamer pipeline as below command:

```console
! qtiqmmfsrc ! qtivtransform ! appsink
```

class "StreamDecode"  decode video stream. refer to inc/StreamDecode.h

# Video Frame Pre-process
The preprocessing of the video frame is resize, normalization and finally convert to Tensor.
We use a video frame with a width/height of 1920*1080 as an example to introduce  how image preprocessing.

- The width and height of the original image before processing is 1920*1080
 
![preprocess-01](https://github.qualcomm.com/storage/user/27150/files/96742ec2-ccbe-4df7-8cd1-81fc35d0d9f8)

- Fill image with fixed width or fixed height. MAX(width, height). the video frame changes from a rectangle to a square

![preprocess-02](https://github.qualcomm.com/storage/user/27150/files/3a104b4e-0657-49fa-9a6d-545eb9cb32cc)

 - Scale to the size required by the model input size 640x640
 - INT8  NHWC image 640x640 convert to NCHW FP32 tensor [1，3，640，640] 

 refer to function
 ```C++
bool ObjectDetectionImpl::PreProcess(const cv::Mat& image)
```
  file: src/Model.cpp

#  AI Model Inference

Use SNPE C++ API for AI inferencing

```C++
bool ObjectDetectionImpl::Detect(shared_ptr<DetectionItem> &item)

```

# Post Process
After the inference is completed, post-processing is decode on the SNPE inteference results

Take the Yolov5 model trained on the different datasets as an example.

SNPE currently does not support 5D operator. It requires specific output nodes before 5D Reshape in convert command. The output nodes can be checked in the https://netron.app/.

To check the output layer nodes, Open the model in the Netron app and click on Conv layer.
![heatmap_model](https://github.qualcomm.com/storage/user/30177/files/7ea84a33-ba60-4b8c-8591-6cc555ed3a0a)


In attached snapshot, the output nodes before 5D is onnx::326 (Conv_198), 365 (Conv_216) and 404 (Conv_234)
![heatmap_nodes](https://github.qualcomm.com/storage/user/30177/files/0fd4147c-762b-4714-b29f-59631321d08b)



## This implementation does below functions on CPU after SNPE inferencing:
* anchorBoxProcess: 
Get raw data from out nodes before 5D (Conv_198, Conv_216, Conv_234), convert to meaning data (scores, class, bounding boxes).
* doNMS: (non-max suppression): remove overlap boxes
* ShowDetectionOverlay: Overlay detection result at output video/Image
![image](https://github.qualcomm.com/storage/user/30177/files/7c1dbc76-5fd8-4ca3-b1eb-33111a9bd45c)


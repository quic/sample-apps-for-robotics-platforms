# SI Solution Design Specification

#  Overview
Below diagram shows the application architecture. This is a simple Producer – Consumer pattern. 
- The application creates a thread to pull up the Camera/RTSP video stream and uses the VPU hardware to decode it. Each video corresponds to a separate thread.
- After the decoding is completed, video frame enters a thread-safe queue.
- When the application starts, a thread is created which is responsible for AI inference, and this thread keeps polling and pop video frames from queue. After getting the latest frames, SM8250 call OpenCV resize, normalization image, convert to tensor then using SNPE for inference.
- After the inference is completed, the CPU performs post-processing on the inference output results and calls the notification callback function to print the result.

![image](https://github.qualcomm.com/storage/user/31436/files/4df62136-e5ca-4fb8-896a-4c12ed04a30c)




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
| labels                   | int    | Number of labels                               |
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
PreProcess function in model implementation file GooglenetSnpe.cpp, GendernetSnpe.cpp and CenterfaceSnpe.cpp
```

#  AI Model Inference

Use SNPE C++ API for AI inferencing

```C++
Detect function in model implementation file GooglenetSnpe.cpp, GendernetSnpe.cpp and CenterfaceSnpe.cpp

```

# Post Process
After the inference is completed, post-processing is decode on the SNPE inteference results

The output nodes can be checked in the https://netron.app/.

## Centerface
![centerface_model](https://github.qualcomm.com/storage/user/30177/files/b7224128-c79a-4b16-85a7-10bc0e634323)


In attached snapshot, the output nodes ::536 (Conv2d_39), 538 (Conv2d_40), 539 (Conv2d_41) and 540 (Conv2d_42)
![centerface_nodes](https://github.qualcomm.com/storage/user/30177/files/7bb64371-46f8-443a-8510-1426f8d54682)

## Age
![image](https://github.qualcomm.com/storage/user/31436/files/f76f695f-9d13-4a88-b954-9c8604e7c5fd)


In attached snapshot, the output node is prob.

## Gender
![image](https://github.qualcomm.com/storage/user/31436/files/c71a9d71-5538-4721-ae71-69a7fd3e7a8a)

In attached snapshot, the output nodes is loss3/loss3_Y(loss3/loss3)

![image](https://github.qualcomm.com/storage/user/31436/files/83c3123b-f2fd-4c1f-b380-0d5c756cac4f)


## Emotion
![image](https://github.qualcomm.com/storage/user/31436/files/ef0a640c-cb78-4ff9-86b3-f5e9824f477b)

In attached snapshot, the output node is 95

![image](https://github.qualcomm.com/storage/user/31436/files/75d743e2-7da8-4eb5-9fc9-930000edcfb0)


## This implementation does below functions on CPU after SNPE inferencing:
* NMS: (non-max suppression): remove overlap boxes
* ShowDetectionOverlay: Overlay detection result at output video/Image

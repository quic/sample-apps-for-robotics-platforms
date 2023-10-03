# SI Solution Design Specification

#  Overview
Below diagram shows the application architecture. This is a simple Producer – Consumer pattern. 
- The application creates a thread to pull up the Camera/RTSP video stream and uses the VPU hardware to decode it. Each video corresponds to a separate thread.
- After the decoding is completed, video frame enters a thread-safe queue.
- When the application starts, a thread is created which is responsible for AI inference, and this thread keeps polling and pop video frames from queue. After getting the latest frames, SM8250 call OpenCV resize, normalization image, convert to tensor then using SNPE for inference.
- After the inference is completed, the CPU performs post-processing on the inference output results and calls the notification callback function to print the result.

![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/9a12ba2c-a284-4c10-bfc2-03ab578debb1)


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
 
![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/844246ae-fd5d-4a8a-b31d-7dccc1e4daaa)

- Fill image with fixed width or fixed height. MAX(width, height). the video frame changes from a rectangle to a square

![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/993e8d83-cf46-420e-9df2-faa694e21b29)

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
![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/efceb717-d184-40d6-8164-01c8796e511d)


In attached snapshot, the output nodes ::536 (Conv2d_39), 538 (Conv2d_40), 539 (Conv2d_41) and 540 (Conv2d_42)
![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/f2547894-dd25-4a2f-a633-33c9b38467e1)

## Age
![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/b5886223-48e6-46da-8a20-0b545c22c57e)

In attached snapshot, the output node is prob.

## Gender
![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/f0deaae0-2029-4f02-885d-9d1fe7f7785a)

In attached snapshot, the output nodes is loss3/loss3_Y(loss3/loss3)

![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/a932ad84-1316-4d82-9283-45fdb325874a)


## Emotion
![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/0bfa9ac6-ed45-4edd-9f08-40daa8441d61)

In attached snapshot, the output node is 95

![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/1ac8b2e7-e15c-4dfc-b455-13dbd9d290de)


## This implementation does below functions on CPU after SNPE inferencing:
* NMS: (non-max suppression): remove overlap boxes
* ShowDetectionOverlay: Overlay detection result at output video/Image

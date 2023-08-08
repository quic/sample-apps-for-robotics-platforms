# SI Solution Design Specification

#  Overview
Below diagram shows the application architecture. This is a simple Producer – Consumer pattern. 
- The application creates a thread to pull up the Camera/RTSP video stream and uses the VPU hardware to decode it. Each video corresponds to a separate thread.
- After the decoding is completed, video frame enters a thread-safe queue.
- When the application starts, a thread is created which is responsible for AI inference, and this thread keeps polling and pop video frames from queue. After getting the latest frames, SM8250 call OpenCV resize, normalization image, convert to tensor then using Qualcomm® Neural Processing SDK for inference.
- After the inference is completed, the CPU performs post-processing on the inference output results and calls the notification callback function to print the result.

![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/03339bf3-6d29-44e4-aa8b-36c7b439efe2)

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
| SkipFrame                | int    | model input height                             |
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
 
![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/7bae7975-fda4-4ff1-95ac-49e6469d4942)

![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/754ef333-b62d-4b49-8fe9-3357cfeb4938)


 - Scale to the size required by the model input size 1088x608
 - INT8  NHWC image 1088x608 convert to NCHW FP32 tensor [1，3，608，1088] 

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
![image](https://github.com/quic/sample-apps-for-robotics-platforms/assets/131336334/bfb6325b-9e21-455a-80a0-8d5fa3d125c7)

The nms_sorted_boxes output is given to the tracker where the bounding boxes are updated. If the bounding box is present in the past frame then tracker ID will be continued or else new unique ID will be assigned.

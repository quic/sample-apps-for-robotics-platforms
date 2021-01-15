![Qualcomm Innovation Center, Inc.](Docs/images/logo-quic-on@h68.png)

This is a repositary of all samples application that can run on RB5. Each sample application has its own folder. An introduction for every sample application is given below. Please follow the README in the respective folders to build, deploy and test. 



## 1. Weston-Client-Application
```
Weston-Sample-code shows how to compile and run a simple client in Weston.
```

## 2. RB5-Information(Device-info)
```
A command based App to get platform information.
from varying target types (GPUs, CPUs, USB, display, sensors and so on).
```

## 3. RB5-Platform(GPIO-samples)
```
A commands based app to get and set GPIO.
For control LED, set GPIO output, Button even catching, etc.
```

## 4. WIFI-On-Boarding
```
WIFI-On-Boarding mainly includes scanning the surrounding hotspots in STA mode, creating a new WIFI connection,
and obtaining the ssid and psk of the currently connected WIFI. After switching the SAP mode, you can create a
new hotspot and get the ssid and psk of the current hotspot.
```

## 5. Tensorflowlite_ROS(ROS-TFLite)
```
This application introduces how to integrate Tensorflow framework into ROS with object detection API on RB5 platform.
It can locate and recognize multiple objects in a single image. The TensorFlow Object Detection API is an open source
framework built on top of TensorFlow that makes it easy to construct, train and deploy object detection models.
ROS provides a publish subscribe communication framework to build distributed computing system simply and quickly. Here are the steps to build.
```

## 6. ROS-Caffe
```
Integrate Coffe model into ROS with Classifier API.
```

## 7. Gstreamer-Applications
```
The purpose of these samples is helping users to learn how to implement the functions of gstreamer on the Qualcomm platform.
```

## 8. C2D-Samples
```
C2D-Sample-code shows picture color-format, picture rotate, picture scale-resize by C2D-Convert-API.
```

## 9. Camera-HAL3-Sample
```
The Camera-HAL3-Sample demo calls the camera through the Camera HAL API to complete the preview capture and video operations.
```

## 10. HexgonSDK-Image-classification
```
This project is designed to show how you can use the Qualcomm® Robotics RB5 development kit, specifically using the Qualcomm® Hexagon™ DSP with Hexagon Vector extensions (HVX), coupled with Hexagon DSP SDK to achieve high speed and high performance on device Machine Learning.
```

## 11. OpenCL-Application
```
OpenCL-Sample-code shows OpenCL from three examples.
"FFT" shows the use of OpenCL for fast Fourier transform;
"Benchmark" shows the reading and writing rate when the memory unit is respectively Byte, KB, MB;
"Matrix_multiply" shows the multiplication of two 20*20 matrices,And print out the results of the two input matrices and the multiplication of the two matrices on the screen
```

## 12. TFLite_Posenet
```
Posenet is to show the effect of pose recognition using Gstreamer commands.
```

## 13. TFLite_Segmentation
```
Segmentation is to show the effect of object segmentation using Gstreamer commands.
```

## 14. OpenMAX-Video
```
This app aims to help users to learn how to encode with h264 and decode the video with h264 on the Qualcom platform through this sample app.
```
## Getting Started
* [Quick start guide](https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/quick-start-guide)
* [Hardware Reference Guide](https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/hardware-reference-guide)
* [Software Reference Manual](https://developer.qualcomm.com/qualcomm-robotics-rb5-kit/software-reference-manual)
* [Discussion Forums](https://developer.qualcomm.com/forum/qdn-forums/hardware/qualcomm-robotics-rb5-development-kit/67886)

## Contributions
Please read our [Contributions Page](CONTRIBUTING.md) for more information on contributing features or bug fixes. We look forward to your participation!

## Team
A community-driven project maintained by Qualcomm Innovation Center, Inc.

## License
Sample applications here are licensed under the BSD 3-clause-Clear “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.

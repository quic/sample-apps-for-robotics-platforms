# Tflite_Posenet
# Overview
Posenet is to show the effect of pose recognition using Gstreamer commands.
## 1. Init
### 1.1  Download the code
```
$ adb shell
$ cd /data
$ git clone https://github.com/quic/sample-apps-for-Qualcomm-Robotics-RB5-platform.git
```
### 1.2  Set up the  environment
You may need to download posenet_mobilenet_v1_075_481_641_quant.tflite models for testing from https://tfhub.dev/tensorflow/lite-model/posenet/mobilenet/float/075/1/default/1 (will be used in "install.h")
then move the posenet_mobilenet_v1_075_481_641_quant.tflite models to /data/tflite_posenet
```
$ cd tflite_posenet
$ sh install.sh
```
### 1.3 Set up the display environment
```
$ source /data/weston.sh &
```
// Set up weston path if it is not already set by weston.sh:
```
sh-4.4# export XDG_RUNTIME_DIR=/usr/bin/weston_socket
```
## 2. Build
```
$ cd /data/tflite_posenet
$ make
```
## 3. Run
```
$ ./tflite_posenet
```

## License
This is licensed under the BSD 3-clause-Clear “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.
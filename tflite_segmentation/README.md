# Tflite_Segmentation
# Overview
Segmentation is to show the effect of object segmentation using Gstreamer commands.
## 1. Init
### 1.1  Download the code
```
$ adb shell
$ cd /data
$ git clone https://github.com/quic/sample-apps-for-Qualcomm-Robotics-RB5-platform.git
```
### 1.2  Set up the  environment
// You may need to download dv3_argmax_int32.tflite models for testing from https://tfhub.dev/tensorflow/lite-model/deeplabv3/1/metadata/2 (will be used in "install.h"), then move the dv3_argmax_int32.tflite models to /data/tflite_segmentation
```
$ cd tflite_segmentation
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
$ cd /data/tflite_segmentation
$ make
```
## 3. Run
```
$ ./tflite_segmentation
```

## License
This is licensed under the BSD 3-clause-Clear “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.
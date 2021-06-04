# RB5 TensorFlow Lite Demo: Object Detection User Guide

## 1. Init and set up the environment

2. Run the script: install.sh
```
   $ cd /data/sample-apps-for-Qualcomm-Robotics-RB5-platform/Gstreamer-Applications/gst_tflite
   $ sh install.sh
```
3. Start the weston display server:

```shell
    sh-4.4# source /data/weston.sh &dd
```

4. Set up weston path manually if it is not already set by weston.sh:

```shell
    sh-4.4# export XDG_RUNTIME_DIR=/usr/bin/weston_socket
```

## 2. Compile the tflite demo app

```shell
    sh-4.4# cd /data/sample-apps-for-Qualcomm-Robotics-RB5-platform/Gstreamer-Applications/gst_tflite/src
    sh-4.4# make
```

## 3. Run the tflite demo app
+ Click left-top button on weston display to open weston-terminal
+ Run ```./tflite_object_detection``` in weston-terminal 

```shell
    sh-4.4# ./tflite_object_detection
```

![Image text](image/gst-tflite2.png)

## License
This is licensed under the BSD 3-clause-Clear  License. Check out the [LICENSE](LICENSE) for more details.

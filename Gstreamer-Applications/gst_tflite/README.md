# RB5 TensorFlow Lite Demo: Object Detection User Guide

## 1. Init and set up the environment

2. Run the script: install.sh
```
   $ cd /data/Gstreamer-Application/gst_tflite
   $ sh install.sh
```
3. Start the weston display server:

```shell
    sh-4.4# source /data/weston.sh &
```

4. Set up weston path manually if it is not already set by weston.sh:

```shell
    sh-4.4# export XDG_RUNTIME_DIR=/usr/bin/weston_socket
```

## 2. Compile the tflite demo app

```shell
    sh-4.4# cd /data/Gstreamer-Application/gst_tflite/src
    sh-4.4# make
```

## 3. Run the tflite demo app

```shell
    sh-4.4# ./tflite_object_detection
```

## License
This is licensed under the BSD 3-clause-Clear  License. Check out the [LICENSE](LICENSE) for more details.

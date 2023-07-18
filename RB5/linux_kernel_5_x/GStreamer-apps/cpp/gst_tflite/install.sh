# Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

# weston.sh: script to start weston display server
cp weston.sh /data

# TFLite object detection model and label files
cp detect.tflite /data/misc/camera
cp labelmap.txt /data/misc/camera


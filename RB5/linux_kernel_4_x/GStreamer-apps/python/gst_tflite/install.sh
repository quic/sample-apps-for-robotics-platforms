# Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

rm -rf detect.tflite labelmap.txt
wget https://raw.githubusercontent.com/quic/sample-apps-for-Qualcomm-Robotics-RB5-platform/master/Gstreamer-Applications/gst_tflite/labelmap.txt
wget https://raw.githubusercontent.com/quic/sample-apps-for-Qualcomm-Robotics-RB5-platform/master/Gstreamer-Applications/gst_tflite/detect.tflite
cp detect.tflite /data/misc/camera
cp labelmap.txt /data/misc/camera


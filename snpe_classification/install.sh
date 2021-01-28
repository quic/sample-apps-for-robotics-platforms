# Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

# weston.sh: script to start weston display server
cp weston.sh /data

# snpe classification model 
cp tensorflow_mobilenet.dlc /data/misc/camera
cp snpe_classification.config /data/misc/camera
cp imagenet_slim_labels.txt /data/misc/camera

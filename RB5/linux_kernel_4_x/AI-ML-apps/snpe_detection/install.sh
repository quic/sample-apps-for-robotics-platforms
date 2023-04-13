# Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

# weston.sh: script to start weston display server
cp weston.sh /data

# snpe detection model 
cp mobilenet-SSD.dlc /data/misc/camera
cp mle_snpe.config /data/misc/camera
cp coco_labels.txt /data/misc/camera

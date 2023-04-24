# Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

#!/bin/bash

[ ! -e /usr/lib/libsync.so ] && ln -s /usr/lib/libsync.so.0 /usr/lib/libsync.so
[ ! -e /usr/lib/libcutils.so ] && ln -s /usr/lib/libcutils.so.0 /usr/lib/libcutils.so
[ ! -e /usr/lib/libutils.so ] && ln -s /usr/lib/libutils.so.0 /usr/lib/libutils.so
[ ! -e /usr/lib/liblog.so ] && ln -s /usr/lib/liblog.so.0 /usr/lib/liblog.so
[ ! -e /usr/lib/libcamera_metadata.so ] && ln -s /usr/lib/libcamera_metadata.so.0 /usr/lib/libcamera_metadata.so


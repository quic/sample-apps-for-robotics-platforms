# Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

# initialize the compiler
CC = gcc

# compiler flags
CFLAGS += $(shell pkg-config --cflags --libs gstreamer-1.0)

# compile
gst-omx_codec_h264: app/src/gst-omx_codec_h264.c
	$(CC) $< -o app/src/$@ $(CFLAGS)

# clean executable file
clean:
	$(RM) app/src/gst-omx_codec_h264

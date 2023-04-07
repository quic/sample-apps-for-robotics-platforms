# Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

# compiler flags
CFLAGS += $(shell pkg-config --cflags --libs gstreamer-1.0)

# compile
gst-codec2_h264: app/src/gst-codec2_h264.c
	$(CC) $< -o app/src/$@ $(CFLAGS)

# clean executable file
clean:
	$(RM) app/src/gst-codec2_h264

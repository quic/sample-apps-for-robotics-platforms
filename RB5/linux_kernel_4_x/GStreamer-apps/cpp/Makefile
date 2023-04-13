# Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

SUBDIRS = \
	gst_audio \
	gst_camera \
	gst_encode_decode \
	gst_gui_app \
	gst_streaming \
	gst_transform \
	gst_alsa/src \
	gst_tflite/src

.PHONY: all clean

all: 
	@list='$(SUBDIRS)'; for subdir in $$list; do \
		$(MAKE) -C $$subdir;\
	done

clean:
	@list='$(SUBDIRS)'; for subdir in $$list; do \
		$(MAKE) -C $$subdir clean;\
	done

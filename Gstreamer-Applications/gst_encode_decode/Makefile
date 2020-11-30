# Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

CC=gcc
FLAGS=`pkg-config --cflags --libs gstreamer-1.0`

SOURCES = \
	usbcam_encode_mp4.c \
	ispcam_encode_mp4.c \
	mp4_decode_playback.c

TARGETS = $(foreach n,$(SOURCES),$(basename $(n)))

all: ${TARGETS}

.PHONY: all clean

${TARGETS}: %:%.c
	$(CC) -Wall $< -o $@ $(FLAGS)

clean:
	rm -f ${TARGETS}

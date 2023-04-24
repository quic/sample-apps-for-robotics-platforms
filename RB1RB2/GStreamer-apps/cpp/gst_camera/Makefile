# Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

FLAGS=`pkg-config --cflags --libs gstreamer-1.0`

SOURCES = \
	usbcam_display.c \
	ispcam_display.c

TARGETS = $(foreach n,$(SOURCES),$(basename $(n)))

all: ${TARGETS}

.PHONY: all clean

${TARGETS}: %:%.c
	$(CC) -Wall $< -o $@ $(FLAGS)

clean:
	rm -f ${TARGETS}

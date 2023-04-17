# Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

CC = gcc
CFLAGS += $(shell pkg-config --cflags --libs gstreamer-1.0)

TARGET = gst_audio_sample

$(TARGET): $(TARGET).c
	$(CC) $< -o $@ $(CFLAGS)

.PHONY: clean

install:
	cp $(TARGET) /usr/bin/

clean:
	rm -f $(TARGET)

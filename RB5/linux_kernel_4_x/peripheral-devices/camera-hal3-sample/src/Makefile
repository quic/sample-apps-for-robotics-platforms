# Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

CC = g++
CFLAGS=
INCLUDES=-I. -I/usr/src/qrb5165-linux-headers-4.19/usr/include -I/usr/include/qrb5165
LFLAGS = -pthread
LIBS = -ldl -lcamera_metadata -lcutils -lutils -llog

SRCS = \
	   CameraHAL3Main.cpp \
	   CameraHAL3Config.cpp \
	   CameraHAL3Buffer.cpp \
	   CameraHAL3Snapshot.cpp \
	   CameraHAL3Device.cpp

OBJS = $(SRCS:.cpp=.o)

EXEC = camera_hal3

all: $(EXEC)

.phony: all depend clean

$(EXEC): $(OBJS)
	$(CC) -o $(EXEC) $(OBJS) $(LFLAGS) $(LIBS)

%.o: %.cpp
	$(CC) $(CFLAGS) $(INCLUDES) -c $<  -o $@

dependency.make:
	touch dependency.make

depend: $(SRCS)
	$(CC) -M $(CFLAGS) $(INCLUDES) $^ > dependency.make

-include dependency.make

clean:
	rm -rf $(EXEC) *.o dependency.make


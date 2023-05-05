# Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

CFLAGS=
INCLUDES=-I. -I/usr/src/qrbx210-linux-headers-5.4/usr/include -I/usr/include/qrbx210 \
	 -I $(SDKTARGETSYSROOT)/usr/include \
	 -I $(SDKTARGETSYSROOT)/usr/include/linux-msm/usr/include
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
	$(CXX) -o $(EXEC) $(OBJS) $(LFLAGS) $(LIBS)

%.o: %.cpp
	$(CXX) $(CFLAGS) $(INCLUDES) -c $<  -o $@

dependency.make:
	touch dependency.make

depend: $(SRCS)
	$(CXX) -M $(CFLAGS) $(INCLUDES) $^ > dependency.make

-include dependency.make

clean:
	rm -rf $(EXEC) *.o dependency.make


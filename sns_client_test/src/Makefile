# Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
#
# SPDX-License-Identifier: BSD-3-Clause-Clear

IDIR =../inc

BIN_PROG = sns_client_test

CC=g++

AM_CPPFLAGS = -Werror                              \
              -Wall                                \
              -Wno-unused-parameter                \
              -Wno-unused-variable                 \
              -fexceptions                         \
              -I/usr/include/qrb5165               \
              -I/usr/include/qrb5165/cutils        \
              -I/usr/include/qrb5165/qmi-framework  

CFLAGS = $(AM_CPPFLAGS) -I$(IDIR)

LDIR =/usr/lib

_SEE_LIBS = libsnsapi.so  \
            libsensorslog.so

SEE_LIBS = $(patsubst %,$(LDIR)/%,$(_SEE_LIBS))

_QMIFRAMEWORK_LIBS = libqmi_common_so.so.1 \
                     libqmi_encdec.so.1    \
                     libqmi_cci.so.1       \
                     libqmi_csi.so.1       \
                     libqmi_sap.so.1
QMIFRAMEWORK_LIBS = $(patsubst %,$(LDIR)/%,$(_QMIFRAMEWORK_LIBS))

_ANDROID_LIBS = liblog.so.0       \
                libcutils.so.0    \
                libtime_genoff.so.1

ANDROID_LIBS = $(patsubst %,$(LDIR)/%,$(_ANDROID_LIBS))

PROTOBUF_LIBS = $(LDIR)/libprotobuf.so.13  \
                -pthread                   \
                $(LDIR)/aarch64-linux-gnu/libpthread.so

requiredlibs = $(SEE_LIBS)               \
               $(ANDROID_LIBS)           \
               $(QMIFRAMEWORK_LIBS)      \
               $(PROTOBUF_LIBS)

LIBS = -lm $(requiredlibs)


_DEPS = sensor_client.h sensor_connection.h
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

OBJ = sensor_client_test.o  \
       sensor_connection.o   \
       sensor_client.o

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

$(BIN_PROG): $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -f *.o $(BIN_PROG)


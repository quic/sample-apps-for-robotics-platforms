# Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

QT += widgets

QT_CONFIG -= no-pkg-config
CONFIG += link_pkgconfig
PKGCONFIG += gstreamer-1.0

HEADERS     = \
              qgstpipeline.h \
              qgstthreadcontroller.h \
              window.h

SOURCES     = main.cpp \
              qgstpipeline.cpp \
              qgstthreadcontroller.cpp \
              window.cpp


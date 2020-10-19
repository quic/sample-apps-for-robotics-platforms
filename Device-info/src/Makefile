# Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

CC = gcc
TARGET_FILE = bin
TAR_INFO = qrb5165_info

$(TAR_INFO): $(TAR_INFO).c
	$(CC) -o ../$(TARGET_FILE)/$(TAR_INFO) $(TAR_INFO).c

clean:
	rm ../$(TARGET_FILE)/*

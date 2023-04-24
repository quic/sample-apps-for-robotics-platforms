# Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

TARGET_FILE = bin
TAR_PLAT = qrb5165_platform

$(TAR_PLAT): $(TAR_PLAT).c
	$(CC) -o ../$(TARGET_FILE)/$(TAR_PLAT) $(TAR_PLAT).c -lpthread

clean:
	rm ../$(TARGET_FILE)/*

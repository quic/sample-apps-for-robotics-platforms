# Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
#
# SPDX-License-Identifier: BSD-3-Clause-Clear

KVERS = $(shell uname -r)

# kernel modules
obj-m := led.o

EXTRA_CFLAGS=-g -O0 -Wno-vla

build: kernel_modules

kernel_modules:
	make -C /usr/src/header M=$(CURDIR) modules

clean:
	make -C /usr/src/header M=$(CURDIR) clean

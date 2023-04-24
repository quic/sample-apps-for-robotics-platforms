# Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

SUBDIRS = \
	audio \
	camera \
	encode_decode \
	streaming \
	transform

.PHONY: all clean

all: 
	@list='$(SUBDIRS)'; for subdir in $$list; do \
		$(MAKE) -C $$subdir;\
	done

clean:
	@list='$(SUBDIRS)'; for subdir in $$list; do \
		$(MAKE) -C $$subdir clean;\
	done

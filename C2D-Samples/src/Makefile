# Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

INC = -I /usr/include/qrb5165 \
      -I /usr/include/qrb5165/ion_headers

all:c2d_test

c2d_test:c2d_test.cpp
	g++ c2d_test.cpp -o c2d_test $(INC) -ldl -lc2dcolorconvert -lion

.PHONY:
	clean
clean:
	rm c2d_test

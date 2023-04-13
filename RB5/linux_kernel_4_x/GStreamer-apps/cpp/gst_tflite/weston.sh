
# Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

#! /bin/bash
mkdir -p /usr/bin/weston_socket
export XDG_RUNTIME_DIR=/usr/bin/weston_socket
export LD_LIBRARY_PATH=/usr/lib:/usr/lib/aarch64-linux-gnu/
weston --tty=1 --connector=29

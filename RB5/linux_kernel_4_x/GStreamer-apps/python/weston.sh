# Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

export XDG_RUNTIME_DIR=/dev/socket/weston
mkdir -p /dev/socket/weston
/usr/bin/weston --tty=1 --idle-time=0 &

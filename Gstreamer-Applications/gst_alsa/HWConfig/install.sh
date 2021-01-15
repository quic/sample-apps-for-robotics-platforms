# Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

#! /bin/bash

chmod 644 qc-alsa-restore.service
chmod 777 qc_alsa_restore.sh
cp qc-alsa-restore.service /lib/systemd/system/
cp qc_alsa_restore.sh /usr/bin/
systemctl enable qc-alsa-restore
sync

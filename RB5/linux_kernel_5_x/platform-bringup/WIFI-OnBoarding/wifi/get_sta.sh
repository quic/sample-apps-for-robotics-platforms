# Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

#!/bin/bash

# Get wireless network card name #
network_card=$(iw dev |grep -o wl.*)

# Determine whether the current connection #
judge=$(iw dev $network_card link)
if [ "$judge" = 'Not connected.' ]; then
    echo -e "\e[31mPlease connect to WiFi first!\e[0m"
else
    #Get current connection SSID
    var1=$(iw dev $network_card link | grep 'SSID:')
    ssid=${var1#*:}
    echo -e "\e[32mCurrent current connection, ssid: $ssid.\e[0m"

    #Get password
    cat /data/misc/wifi/wpa_supplicant.conf | grep -A10 $ssid | grep -m 1 psk=
fi

# Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

#!/bin/bash

# Get wireless network card name #
network_card=$(iw dev | grep -o wl.*)

# judge wpa_supplicant is already running #
judge_conf=$(ps -aux | grep wpa_supplicant | grep -v grep)
if [ -z "$judge_conf" ]; then
    # Start wpa_supplicant #
    wpa_supplicant -i$network_card -Dnl80211 \
        -c /data/misc/wifi/wpa_supplicant.conf \
        -O /data/misc/wifi/sockets -B
else
    echo -e "\e[32mWpa_supplicant is already running!\e[0m"
fi

# List saved SSID #
wpa_cli -i $network_card list_network -p /data/misc/wifi/sockets

# Judge whether the list of WiFi saved is empty #
list_result=$(wpa_cli -i $network_card list_network -p /data/misc/wifi/sockets)
judge_result=`echo "$list_result" | sed -n '2p'`

if [ "$judge_result" = "" ]; then
    echo -e "\e[31mWiFi list is empty, please add it first!\e[0m"
else
    read -p "Please enter the 'network id' or 'all' to delete:" num

    # Delete saved WiFi #
    ret_rm=$(wpa_cli -i $network_card remove_network $num -p /data/misc/wifi/sockets)
    echo -e "\e[32mRemove #$num WIFI $ret_rm\e[0m"

    # Update configuration table #
    ret_save=$(wpa_cli -i $network_card save_config -p /data/misc/wifi/sockets)
    echo -e "\e[32mUpdate configuration table $ret_save\e[0m"

    # List saved SSID #
    wpa_cli -i $network_card list_network -p /data/misc/wifi/sockets
fi

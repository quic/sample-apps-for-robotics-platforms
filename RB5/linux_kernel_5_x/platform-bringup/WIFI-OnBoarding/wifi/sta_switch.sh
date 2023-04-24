# Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

#!/bin/bash

# Get wireless network card name #
network_card=$(iw dev | grep -o wl.*)

# Judge whether the input is consistent with #
read -p "Switch Of STA [on/off]?" flag
while [ "$flag" != 'on' -a "$flag" != 'off' ]; do
    echo -e "\e[31mPlease enter the correct command!\e[0m"
    read -p "Please enter again:" flag
done

# Configure wpa_supplicant.conf file #
judge_conf=$(cat /data/misc/wifi/wpa_supplicant.conf | grep update_config=1)
if [ -z "$judge_conf" ]; then
    sed '3iupdate_config=1' /data/misc/wifi/wpa_supplicant.conf -i
    echo -e "\e[32mwpa_supplicant.conf modification succeeded!\e[0m"
else
    echo -e "\e[32mThe configuration file has been configured successfully!\e[0m"
fi

if [ "$flag" = 'on' ]; then
    # Initialization environment #
    pgrep hostapd | xargs kill -9 &>/dev/null
    pgrep dnsmasq | xargs kill -9 &>/dev/null

    # judge wpa_supplicant is already running #
    judge_conf=$(ps -aux | grep wpa_supplicant | grep -v grep)
    if [ -z "$judge_conf" ]; then
        # Start wpa_supplicant #
        wpa_supplicant -i$network_card -Dnl80211 -c /data/misc/wifi/wpa_supplicant.conf -O /etc/wlan/sockets -B
    else
        echo -e "\e[32mWpa_supplicant is already running!\e[0m"
    fi

    # List saved SSID #
    wpa_cli -i $network_card list_network -p /etc/wlan/sockets 

    # Judge whether the WiFi save list is empty #
    list_result=$(wpa_cli -i $network_card list_network -p /etc/wlan/sockets )
    judge_result=`echo "$list_result" | sed -n '2p'`

    if [ "$judge_result" = "" ]; then
        echo -e "\e[31mWiFi list is empty, please add it first!\e[0m"
    else
        # choose seved SSID #
        read -p "Please choose SSID's network id:" num
        ret_select=$(wpa_cli -i wlan0 select_network $num -p /etc/wlan/sockets )
        echo -e "\e[32mSelect WIFI $ret_select.\e[0m"

        # enable SSID #
        ret_enable=$(wpa_cli -i $network_card enable_network $num -p /etc/wlan/sockets )
        echo -e "\e[32mEnable WIFI $ret_enable.\e[0m"
    fi
else
    pgrep wpa_supplicant | xargs kill -9 &>/dev/null
    echo -e "\e[32mSTA Mode Closed Successfully!\e[0m"
fi

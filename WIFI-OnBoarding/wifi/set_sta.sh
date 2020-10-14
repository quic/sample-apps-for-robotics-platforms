# Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

#!/bin/bash

# Get Wireless Network Card Name #
network_card=$(iw dev | grep -o wl.*)

# Enable network card #
ifconfig $network_card up

# Initialization environment #
pgrep wpa_supplicant | xargs kill -9 &>/dev/null

# Start wpa_supplicant #
wpa_supplicant -iwlan0 -Dnl80211 -c /data/misc/wifi/wpa_supplicant.conf -O /data/misc/wifi/sockets -B

# Add network #
ret_NetID=$(wpa_cli -i $network_card add_network -p /data/misc/wifi/sockets)
echo -e "\e[32mThe current network label is $ret_NetID.\e[0m"

# Read user input WiFi_SSID #
read -p "Please enter the new WIFI_SSID:" new_ssid
while [ -z $new_ssid ]; do
    echo -e "\e[31mPlease do not enter an empty ssid!\e[0m"
    read -p "Please enter again:" new_ssid
done

ret_WifiID=$(wpa_cli -i $network_card set_network $ret_NetID ssid  '"'$new_ssid'"' -p /data/misc/wifi/sockets)
if [ "$ret_WifiID" = 'OK' ]; then
    echo -e "\e[32mSet the new WIFI_SSID $ret_WifiID!\e[0m"

    # Read user input WiFi_PSK #
    read -p  "Please enter the new WIFI_PSK:" new_psk
    while [ ${#new_psk} -lt 8 ]; do
        echo -e "\e[31mPlease enter a password of at least 8 digits!\e[0m"
        read -p "Please try again:" new_psk
    done

    ret_WifiPSK=$(wpa_cli -i $network_card set_network $ret_NetID psk  '"'$new_psk'"' -p /data/misc/wifi/sockets)

    # Determine whether the password meets the input requirements, otherwise delete #
    if [ "$ret_WifiPSK" = 'OK' ]; then
        echo -e "\e[32mSet the new WIFI_PSK $ret_WifiPSK!\e[0m"

        # Determine whether SSID and PSK already exist in the configuration table #
        judge_ssid=$(cat /data/misc/wifi/wpa_supplicant.conf | grep -o ssid=\"$new_ssid\")
        judge_psk=$(cat /data/misc/wifi/wpa_supplicant.conf | grep -A10 $new_ssid | grep -m 1 psk= | grep -o psk=\"$new_psk\")

        if [ "$judge_ssid" == "ssid=\"$new_ssid\"" ] && [ "$judge_psk" == "psk=\"$new_psk\"" ]; then
            echo -e "\e[31mThe SSID and PSK currently entered already exist!\e[0m"

            # Delete the SSID and PSK currently entered #
            ret_rm=$(wpa_cli -i $network_card remove_network $ret_NetID -p /data/misc/wifi/sockets)
            echo -e "\e[32mDelete the SSID and PSK just entered $ret_rm!\e[0m"

            # List current saved hotspot information #
            wpa_cli -i $network_card list_network -p /data/misc/wifi/sockets

            # Query the WiFi number saved in the configuration table #
            ret_list=$(wpa_cli -i $network_card list_network -p /data/misc/wifi/sockets)
            judge_list=`echo "$ret_list" |  sed -n "/$new_ssid/p" | awk '{print $1}'`
            # Enable saved WiFi #
            ret_enable=$(wpa_cli -i $network_card enable_network $judge_list -p /data/misc/wifi/sockets)
            echo -e "\e[32mEnable #$judge_list WIFI as you think $ret_enable!\e[0m"
        else
            # Enabling network #
            ret_enable_network=$(wpa_cli -i $network_card enable_network "$ret_NetID"  -p /data/misc/wifi/sockets)
            echo -e "\e[32mEnable #$ret_NetID WIFI successfuly $ret_enable_network!\e[0m"

            # List current saved hotspot information #
            wpa_cli -i $network_card list_network -p /data/misc/wifi/sockets

            #Save to configuration table
            ret_dave_config=$(wpa_cli -i  $network_card save_config -p /data/misc/wifi/sockets)
            echo -e "\e[32mSave to configuration table $ret_dave_config!\e[0m"

            # Automatic acquisition IP #
            #udhcpc -i $network_card -q -n -R
        fi

    else
        # Delete current hotspot #
        ret_remove_network=$(wpa_cli -i $network_card remove_network "$ret_NetID!" -p /data/misc/wifi/sockets)
        echo -e "\e[31mDeleted the current network label $ret_NetID\e[0m"
    fi

else
    # Delete current hotspot #
    ret_remove_network=$(wpa_cli -i $network_card remove_network "$ret_NetID" -p /data/misc/wifi/sockets)
    echo -e "\e[31mDeleted the current network label $ret_NetID!\e[0m"
fi

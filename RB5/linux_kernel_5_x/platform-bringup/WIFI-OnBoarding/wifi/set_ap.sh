# Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

#!/bin/sh

# Initialization environment #
pgrep hostapd | xargs kill -9 &>/dev/null
pgrep dnsmasq | xargs kill -9 &>/dev/null

# Get network card name #
network_card=$(iw dev | grep -o wl.*)

ifconfig $network_card down

# set the new ssid #
read -p "Please enter the new Hostpot_SSID:" hostpot_ssid

# Determine whether the ssid meets the requirements #
while [ -z $hostpot_ssid ]; do
    echo -e "\e[31mPlease do not enter an empty ssid!\e[0m"
    read -p "Please enter again:" hostpot_ssid
done

# set the new wpa_passphrase #
read -p "Please enter the new Hostpot_PSK:" hostpot_psk

# Determine whether the password meets the requirements #
while [ ${#hostpot_psk} -lt 8 ]; do
    echo -e "\e[31mPlease enter a password of at least 8 digits!\e[0m"
    read -p "Please try again:" hostpot_psk
done

# Save ssid and psk to the configuration table #
sed -i 's/^ssid=.*/ssid='"$hostpot_ssid"'/g' /etc/hostapd/hostapd.conf
sed -i 's/^wpa_passphrase=.*/wpa_passphrase='"$hostpot_psk"'/g' /etc/hostapd/hostapd.conf

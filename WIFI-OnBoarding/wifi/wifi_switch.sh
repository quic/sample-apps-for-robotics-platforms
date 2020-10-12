# Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause

#!/bin/bash

# Judge whether the input is correct #
read -p "Switch Of WLAN Module [on/off]?" flag
while [ "$flag" != 'on' -a "$flag" != 'off' ]; do
    echo -e "\e[31mPlease enter the correct command!\e[0m"
    read -p "Please enter again:" flag
done

# Get wireless network card name #
network_card=$(iw dev | grep -o wl.*)
echo -e "\e[32mCurrent wireless network card is $network_card.\e[0m"

if [ "$flag" = 'on' ]; then
    # Start wireless network card #
    ifconfig $network_card up
    echo -e "\e[32mThe $network_card opened successfully!\e[0m"
else
    # Initialization environment #
    pgrep wpa_supplicant | xargs kill -9 &>/dev/null
    pgrep hostapd | xargs kill -9 &>/dev/null
    pgrep dnsmasq | xargs kill -9 &>/dev/null

    # Close wireless network card #
    ifconfig wlan0 down
    echo -e "\e[32mThe $network_card closed successfully!\e[0m"
fi

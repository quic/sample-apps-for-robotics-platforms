# Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

#!/bin/sh

# Initialization environment #
pgrep wpa_supplicant | xargs kill -9 &>/dev/null
pgrep hostapd | xargs kill -9 &>/dev/null
pgrep dnsmasq | xargs kill -9 &>/dev/null

# Read user selection #
read -p "switch of SAP mode [on/off]?" flag
while [ "$flag" != 'on' -a "$flag" != 'off' ]; do
    echo -e "\e[31mPlease enter the correct command!\e[0m"
    read -p "Please enter again:" flag
done

# Get network card name #
network_card=$(iw dev | grep -o wl.*)

if [ "$flag" = 'on' ]; then
    # Enable network card #
    ifconfig $network_card 192.168.3.1 up

    # Start-up hostapd #
    hostapd -dddd -B /etc/hostapd/hostapd.conf

    # Stop systemd-resolved service #
    #systemctl stop systemd-resolved

    # Start-up dnsmasq #
    dnsmasq -i wlan0 -l /data/dnsmasq.leases --no-daemon --no-resolv \
                --no-poll --dhcp-range=192.168.3.100,192.168.3.200,1h &

    # Forwarding configuration #
    echo 1 > /proc/sys/net/ipv4/ip_forward
    iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
else
    pgrep hostapd | xargs kill -9 &>/dev/null
    pgrep dnsmasq | xargs kill -9 &>/dev/null

    ifconfig $network_card down
    ifconfig $network_card up

    echo -e "\e[32mSAP mode closed successfully!\e[0m"
fi

/*
* Copyright (c) 2020, 2022 Qualcomm Innovation Center, Inc.  All Rights Reserved.
*
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include <stdio.h>
#include <stdlib.h> //exit()  system()

/*STA mode to obtain hot spot information around*/
void get_hotspot_info() {
    system("iw dev wlan0 scan | grep  -e SSID: -e signal:");
}

/*Set SSID and PSK in STA mode*/
void set_sta_idpsk() {
    system("sh ./set_sta.sh");
}

/*STA mode gets the current SSID and PSK*/
void get_sta_idpsk() {
    system("sh ./get_sta.sh");
}

/*STA mode switching AP mode*/
void sta_switch_ap() {
    system("sh ./ap_switch.sh");
}

/*AP mode gets the current SSID and PSK*/
void get_ap_idpsk() {
    system("cat /etc/hostapd/hostapd.conf | grep -e '^ssid=' -e '^wpa_passphrase='");
}

/*AP mode setting new SSID and PSK*/
void set_ap_idpsk() {
    system("sh ./set_ap.sh");
}

/*switch of wifi*/
void wifi_switch() {
    system("sh ./wifi_switch.sh");
}

/*sta mode of switch*/
void sta_switch() {
    system("sh ./sta_switch.sh");
}

/*rmove saved wifi*/
void rm_sta_idpsk() {
    system("sh ./rm_sta.sh");
}

int main() {
    system("clear");
    /*Menu*/
    while (1) {
        int num = 0;

        printf("\033[1;32m*~*~*~*~*~*~*~*~*~WIFI-On-Boarding~*~*~*~*~*~*~*~*\033[0m\n");
        printf("1. WLAN Module Switch (on/off).\n");
        printf("2. STA Mode: Scan The Available Hotspots.\n");
        printf("3. STA Mode Switch (on/off).\n");
        printf("4. STA Mode: Set The New SSID And PSK.\n");
        printf("5. STA Mode: Get Current SSID And PSK.\n");
        printf("6. STA Mode: Remove Saved WIFI.\n");
        printf("7. SAP Mode Switch (on/off).\n");
        printf("8. SAP Mode: Set The New SSID And PSK.\n");
        printf("9. SAP Mode: Get Current SSID And PSK.\n");
        printf("0. Exit.\n");
        printf("\033[1;32m*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~*~\033[0m\n");
        printf("\033[1;32mPlease enter the operation number to be performed:\033[0m\n");
        scanf("%d",&num);

        switch (num) {
        case 0:
            exit(0);
        break;

        case 1:
            system("clear");
            wifi_switch();
        break;

        case 2:
            system("clear");
            get_hotspot_info();
        break;

        case 3:
            system("clear");
            sta_switch();
        break;

        case 4:
            system("clear");
            set_sta_idpsk();
        break;

        case 5:
            system("clear");
            get_sta_idpsk();
        break;

        case 6:
            system("clear");
            rm_sta_idpsk();
        break;

        case 7:
            system("clear");
            sta_switch_ap();
        break;

        case 8:
            system("clear");
            set_ap_idpsk();
            sta_switch_ap();
        break;

        case 9:
            system("clear");
            get_ap_idpsk();
        break;

        default:
            printf("\033[1;31mPlease enter the correct operation number.Thanks!\033[0m\n");
        }
    }
    return 0;
}

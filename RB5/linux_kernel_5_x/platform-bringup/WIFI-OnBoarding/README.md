# WIFI-On-Boarding
## Overview
WIFI-On-Boarding mainly includes scanning the surrounding hotspots in STA mode, creating a new WIFI connection, and obtaining the ssid and psk of the currently connected WIFI. After switching the SAP mode, you can create a new hotspot and get the ssid and psk of the current hotspot.

## 1. Init:
Install RB5 LU SDK and source the environment on PC
Download sample code on PC

## 2. Compile the demo app
```
$ cd WIFI-OnBoarding/wifi
$ $CC wifi.c -o wifi
```

## 3. Push the binary to the device and run
```
$ adb disable-verity
$ adb reboot
$ adb wait-for-device root
### The above three steps only need to be operated once and will always be valid.

$ adb shell mount -o remount,rw /
$ adb push wifi /data
$ adb shell
$ cd /data
$ chmod 777 wifi
$ mkdir /etc/wlan
$ ./wifi
```

## 4. Result :
Enter the corresponding operation number according to the menu reminder.

![Image text](image/new_menu.png)

### (1) WiFi module switch (on/off).-->wifi_switch.sh
User input: 1

![Image text](image/input_1.png)

If you choose 'on'/'off', you can see something similar to the following.

User input: on

![Image text](image/wifi_on.png)

User input: off

![Image text](image/wifi_off.png)

Check: you can check the status of wlan0 through commands "$ ifconfig".

Note: of course, you must turn on wlan0 for the following operations.

### (2) STA mode:scan the available hotspot and return signal strength.-->wifi.c
User input: 2

Check: if successful, you can see the same situation as the picture below.

![Image text](image/new_scan.png)

### (3) STA mode switch (on/off).-->sta_switch.sh
User input: 3

User input: on

User input: "network id"

![Image text](image/sta_on.png)

Check:If successful, the board can be connected to the WiFi of your choice

User input: off

![Image text](image/sta_off.png)

Check:The board will disconnect WiFi

Note:You can also use this function to select a saved WiFi

### (4) STA mode:set the new SSID and PSK.-->set_sta.sh
User input: 4

User input: "New SSID"

User input: "New PSK"

![Image text](image/new_set_sta.png)

Check: if successful, the board will switch to the new WiFi.

If you create the same SSID and PSK, you will be rejected.

![Image text](image/set_same_sta.png)

Check: it connects to the saved WFI.

### (5) STA mode:get current SSID and PSK.-->get_sta.sh
User input: 5

![Image text](image/new_get_sta.png)

If you're not connected to WiFi to get it.

![Image text](image/fail_get_sta.png)

### (6) STA Mode: Remove Saved WIFI.-->rm_sta.sh
User input: 6

User input: "network id"

![Image text](image/rm_save_sta.png)

Or user input: all

![Image text](image/rm_all_sta.png)

### (7) SAP mode switch (on/off).-->ap_switch.sh
User input: 7

User input: on

![Image text](image/ap_on.png)

Check: when a device is connected, you can see the following log.Of course, you can use your phone to connect to this hotspot.

![Image text](image/ap_connect.png)

User input: off

![Image text](image/ap_off.png)

Check: other devices disconnect the hot spots from the board

### (8) SAP mode:set the new SSID and PSK.-->set_ap.sh
User input: 8

User input: "new Hostpot_SSID"

User input: "new Hostpot_PSK"

User input: on

![Image text](image/new_set_ap.png)

Check: if successful, you can see the same display as function 6.

![Image text](image/ap_on.png)

### (9) SAP mode:get current SSID and PSK.-->wifi.c
User input: 9

If successful, you can see the current hotspot SSID and PSK

![Image text](image/new_get_ap.png)

## 5. matters needing attention

1. When using the AP function, some logs will be printed, but it does not hinder the use of other functions. You can continue to enter the operation number.

2. Set new SSID cannot be empty. The length of PSK must be equal or greater than 8.

3. I wish you a good time :)

## License
This is licensed under the BSD 3-Clause-Clear “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.

# Video Streaming

These samples show two different ways to stream video on network.

The first sample creates a TCP streaming server with gstreamer.

The second sample demonstrates how to run a RTSP server on the board.


## TCP Streaming Server

```tcp_server``` streams video on network through TCP.

Usage: tcp_server [camera id: 0|1|2|3] IP port

 IP: [x.x.x.x] : IP address of the board. It maybe through eth0 or wlan0.

### Start Streaming:

Example of streaming video from camera 0 on port 5000, IP of the board is 192.168.3.5:
``` bash
$ adb disable-verity
$ adb reboot
$ adb wait-for-device root
### The above three steps only need to be operated once and will always be valid.

$cd /data/gstreamer-applications/streaming
$ ./tcp_server 0 192.168.3.5 5000
```

#### Note:

+ Use isp-camera 
+ The board may have different network interfaces enabled, e.g. eth0 and wlan0. Therefore, please use ```ifconfig``` to get IP address of the board and specify the IP address as a parameter manually. 
+ The network cable connects the board and the PC. (DIP_SW_3 SW2-0ff) 
+ Configure the IP address and gateway.

### Client Player:

In client computer:
Configure the IP address and gateway to be on the same LAN as the board.

#### Command Line Example:

```bash
vlc tcp://192.168.3.5:5000/
```

#### VLC Application

+ File -> Open Network Stream
+ Input URL: ```tcp://192.168.3.5:5000/``` for example.
+ Click 'Play'

## RTSP Server


### Start RTSP Server

Start tcp and adb connection:
$ adb forward tcp:8900 tcp:8900

Stream camera video on RTSP:

```bash
$ adb shell
$ cd /data
$ gst-rtsp-server -p 8900 -m /live "( udpsrc name=pay0 port=8554 caps=\"application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96\" )"
```
Wait for setup to be done until the following is shown:
	Stream ready at rtsp://127.0.0.1:8900/live

Open another terminal:
```
$ adb shell
$ export XDG_RUNTIME_DIR=/run/user/root && gst-launch-1.0 -e qtiqmmfsrc ! video/x-raw\(memory:GBM\),width=1280,height=720,framerate=30/1 ! queue ! qtic2venc ! queue ! h264parse config-interval=-1 ! rtph264pay pt=96 ! udpsink host=127.0.0.1 port=8554
```

### Client Player:

In client computer:

#### Command Line Example:

```bash
vlc rtsp://127.0.0.1:8900/live
```

#### VLC Application

+ File -> Open Network Stream
+ Input URL: ```rtsp://127.0.0.1:8900/live```
+ Click 'Play'

#### Note:

+ Please use ```ifconfig``` on the board to get IP address of the board.

## License
This is licensed under the BSD 3-Clause-Clear “New” or “Revised” License. Check out the [LICENSE](../LICENSE) for more details.

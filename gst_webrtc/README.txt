##############################################################################
The patch 001-webrtc-modified-rb5.patch is on the following source:
https://gitlab.freedesktop.org/gstreamer/gst-examples/-/tree/master/webrtc/sendonly

1. Removes TURN servers (local signalling over websockets)
2. Removes audio. 
3. Configures transcievers to be video stream only. 
4. Uses the Camera ISP via GST QMMF to ensure HW encoding.
5. Reconciles the webrtc code to the webrtc headers we have.
##############################################################################

1. From your host go to adb
   $adb shell

2. Create a webrtc directory
   #mkdir -p /data/webrtc

3. Install the requisite plugins
   #apt-get update
   #apt-get install -y gstreamer1.0-tools gstreamer1.0-nice gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-plugins-good libgstreamer1.0-dev git libglib2.0-dev libgstreamer-plugins-bad1.0-dev libsoup2.4-dev libjson-glib-dev

4. Push the source
   $adb push webrtc/sendonly /data/webrtc/
   $adb shell sync
   $adb shell
   #cd /data/webrtc/sendonly

5. Apply the patch
   #patch < 001-webrtc-modified-rb5.patch
   #make
   #./webrtc-unidirectional-h264

######## VERY_VERY_IMPORTANT (this took quite some time) ##############

6. Open Chrome browser. Depending on the version you may have to do 1 or 2
   1. "Temporarily unexpire M85 flags" (#temporary-unexpire-flags-m86) set to Enabled
   2. "Temporarily unexpire M90 flags" (#temporary-unexpire-flags-m90) set to Enabled
   3. chrome://flags/#enable-webrtc-hide-local-ips-with-mdns (Set it to DISABLED. IT WON'T WORK IF YOU DO NOT DO THIS :-))   
 
7. Re-start chrome

8. Open tab and point to http://<rb5_ip_address>:57778
   <stream should begin and you can see ICE exchanges on the console>
   <you can view the stats of webrtc on the browser using: chrome://webrtc-internals>

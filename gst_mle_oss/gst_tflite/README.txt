1. adb shell [Please do this only once]
   # apt-get install cmake
   # mkdir -p ~/DEVELOPMENT/
   # cd /root/DEVELOPMENT/
   # git clone <this_repo_url>
   # cd gst_tflite
2. Now you can perform the on-device compilation
   $ adb shell
   # cd /root/DEVELOPMENT/gst_tflite
   # cmake . 
   # make
   # make install
   ...

3. Run gst-inspect-1.0 (You should not see any blacklisted plugins)
   #gst-inspect-1.0

4. Please use the steps to setup the weston_server (in Quick start guide) and then run tflite inferencing via gst launch.
   $ adb shell 
   #export XDG_RUNTIME_DIR=/usr/bin/weston_socket && gst-launch-1.0 qtiqmmfsrc ! video/x-raw, format=NV12,width=1280,height=720,framerate=30/1,camera=0 ! qtimletflite config=/data/misc/camera/mle_tflite.config model=/data/misc/camera/detect.tflite labels=/data/misc/camera/labelmap.txt postprocessing=detection ! queue !  qtioverlay! waylandsink fullscreen=true async=true enable-last-sample=false

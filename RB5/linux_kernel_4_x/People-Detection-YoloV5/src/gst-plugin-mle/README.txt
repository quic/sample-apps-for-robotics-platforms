1. Download SNPE SDK v1.68.0 from here (On device compilation has been tested on v1.68)
   https://developer.qualcomm.com/downloads/qualcomm-neural-processing-sdk-ai-v1680?referrer=node/34505

2. Please push the SNPE libraries from extracted SNPE sdk into the requisite folder . 
   $ cd {}\snpe-1.68.0\snpe-1.68.0.3932\
   $ 
   $ adb push lib\aarch64-ubuntu-gcc7.5\. /usr/lib/
   $ adb push lib\aarch64-ubuntu-gcc7.5\libsnpe_dsp_domains_v2.so /usr/lib/rfsa/adsp/
   $ adb push lib\dsp\. /usr/lib/rfsa/adsp/ 

3. Download and install fastcv sdk
   $ Install Java on host
   * apt install default-jdk
   $ Download and Install fastcv sdk on host
   * https://developer.qualcomm.com/downloads/qualcomm-computer-vision-sdk-v171-linux-embedded-linux-installer?referrer=node/7332
   * chmod +x fastcv-installer-linuxembedded-1-7-1.bin
   * ./fastcv-installer-linuxembedded-1-7-1.bin
   $ Push the fastcv package to device
   adb push <fastcv package path> /root/DEVELOPMENT/

4. Download and Extract the snpe sdk zip file on your target machine mentioned in step 1.

5. adb shell [Please do this only once]
   # apt-get install cmake
   # mkdir -p ~/DEVELOPMENT/
   # cd /root/DEVELOPMENT/
   # git clone git clone https://github.qualcomm.com/khrahul/CE2.0_SI_Solutions.git -b cross-compile
   # cd CE2.0_SI_Solutions/W05/YoloV5/src/gst-plugin-mle
   # mkdir build
   # cd build

6. Top Level structure for your reference on your RB5.
   
   /root/DEVELOPMENT/gst-plugin-mle
   |
   |---CMakeLists.txt
   |---mle_engine
   |---mle_gst_snpe
   |---build

7. Now you can perform the on-device compilation
   # cmake -DCMAKE_VERBOSE_MAKEFILE=1 -DSNPE_ENABLE:BOOL=ON -DSNPE_SDK_BASE_DIR=<SNPE Directory path>/snpe-1.68.0.3932 -DFASTCV_SDK_DIR=<FastCV directory path> ..
   # make
   # make install

8. Push the model, config and label files
   $ mkdir /data/misc/camera
   $ cd /root/DEVELOPMENT/CE2.0_SI_Solutions/model
   $ cp yolov5m_quant.dlc /data/misc/camera
   $ cp mle_snpeyolov5m_quant.config /data/misc/camera
   $ cp coco_labels.txt /data/misc/camera

9. Run the gstreamer pipeline:
   $ export XDG_RUNTIME_DIR="/usr/bin/weston_socket"
   $ mkdir -p $XDG_RUNTIME_DIR
   $ chmod 0700 $XDG_RUNTIME_DIR
   $ /usr/bin/weston --tty=1 --connector=29 &

   $ gst-launch-1.0 qtiqmmfsrc ! video/x-raw\(memory:GBM\), format=NV12, width=1280, height=720, framerate=30/1 ! queue ! qtimlesnpe config=/data/misc/camera/mle_snpeyolov5m_quant.config postprocessing=yolov5detection ! queue ! qtioverlay bbox-color=0xFF0000FF ! waylandsink  width=1920 height=1080 async=true sync=false enable-last-sample=false

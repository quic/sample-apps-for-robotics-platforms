1. Download SNPE SDK v1.43.0 from here (On device compilation has been tested on v1.43)
   https://developer.qualcomm.com/docs/snpe/overview.html

2. Please push the stub and skeleton from the above extracted SNPE sdk into the requsite folder (on your host pc). 
   $ cd {}\snpe-1.43.0\snpe-1.43.0.2307\lib
   $ 
   $ adb push lib\aarch64-ubuntu-gcc7.5\lib* /usr/lib/
   $ adb push lib\aarch64-ubuntu-gcc7.5\libsnpe_dsp_domains_v2.so /usr/lib/rfsa/adsp/
   $ adb push dsp\libsnpe_dsp_v66_domains_v2_skel.so /usr/lib/rfsa/adsp/ 

3. adb shell [Please do this only once]
   # apt-get install cmake
   # mkdir -p ~/DEVELOPMENT/
   # cd /root/DEVELOPMENT/
   # git clone <this_repo_url>
   # cd gst_snpe

4. Extract the snpe sdk zip file on your host PC.
5. Create a snpe-sdk folder with the structure as shown below. 
   * Copy the libraries from lib/aarch64-ubuntu-gcc7.5/ into the new lib folder as shown below.
   * Copy the include from include into the new include folder as shown below.   
6. Top Level structure for your reference on your RB5.
   
   /root/DEVELOPMENT/gst_snpe
   |
   |---CMakeLists.txt
   |---mle_engine
   |---mle_gst_snpe
   |---snpe-sdk
        |---include
            |---zdl
                |--*
        |---lib
            |---libSNPE.so
            |---*
7. Now you can perform the on-device compilation
   $ adb shell
   # cd /root/DEVELOPMENT/gst_snpe
   # cmake -DSNPE_SDK_BASE_DIR=/root/DEVELOPMENT/gst_snpe/snpe-sdk . 
   # make
   # make install
   ...

8. Run gst-inspect-1.0 (You should not see any blacklisted plugins)
   #gst-inspect-1.0

9. Please get the appropriate DLC models and the label map and configure it in the config file. A sample mle_snpe.config file is present in the snpe folder. For detection case make sure the following is configured and the DLC and label maps are configured. 
   output_layers = < "add_6", "Postprocessor/BatchMultiClassNonMaxSuppression" >

10. Push the models and the label map to the right directory configured in the mle_snpe.config file. 

11. Please use the steps to setup the weston_server (in Quick start guide) and then run snpe inferencing via gst launch and run the command below.

12. $adb shell 
   #export XDG_RUNTIME_DIR=/usr/bin/weston_socket && gst-launch-1.0 qtiqmmfsrc ! video/x-raw, format=NV12, width=1280, height=720, framerate=30/1, camera=0 ! qtimlesnpe config=/data/misc/camera/mle_snpe.config postprocessing=detection ! queue ! qtioverlay ! waylandsink fullscreen=true enable-last-sample=false

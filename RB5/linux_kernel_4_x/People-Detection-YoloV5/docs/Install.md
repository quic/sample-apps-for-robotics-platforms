## Please follow the steps to compile and deploy the qtimlesnpe plugin on target device

#### 1. Download Qualcomm® Neural Processing SDK v1.68.0 from here:

https://developer.qualcomm.com/downloads/qualcomm-neural-processing-sdk-ai-v1680?referrer=node/34505

#### 2. Please push the SDK libraries extracted from SDK into the requisite folder . 
```console
cd {}\snpe-1.68.0\snpe-1.68.0.3932\
```
```console
adb push lib\aarch64-ubuntu-gcc7.5\. /usr/lib/
```
```console
adb push lib\aarch64-ubuntu-gcc7.5\libsnpe_dsp_domains_v2.so /usr/lib/rfsa/adsp/
```
```console
adb push lib\dsp\. /usr/lib/rfsa/adsp/ 
```
```console
adb push bin\aarch64-ubuntu-gcc7.5\snpe-net-run /usr/bin/
```

Verify the snpe version:
```console
adb shell
```
```console
chmod +x /usr/bin/snpe-net-run
```
```console
snpe-net-run --version
```
Check if it matches with the SDK version. If not matches, then please push the libraries again and verify.

#### 3. Download and install fastcv sdk
##### Install Java on host
```console
apt install default-jdk
```
##### Download and Install fastcv sdk on host (Linux). 
* https://developer.qualcomm.com/downloads/qualcomm-computer-vision-sdk-v171-linux-embedded-linux-installer?referrer=node/7332
* chmod +x fastcv-installer-linuxembedded-1-7-1.bin
* ./fastcv-installer-linuxembedded-1-7-1.bin

If using Windows, Please download the Windows package from the website.
https://developer.qualcomm.com/software/qualcomm-computer-vision-sdk/tools

##### Push the fastcv package to device
```console
adb push <fastcv package path> /root/DEVELOPMENT/
```

#### 4. Download and Extract the SDK zip file on your target machine mentioned in step 1.

#### 5. adb shell [Please do this only once]
```console
apt-get install cmake
```
```console
mkdir -p ~/DEVELOPMENT/
```
```console
cd /root/DEVELOPMENT/
```
```console
git clone https://github.qualcomm.com/khrahul/CE2.0_SI_Solutions.git -b cross-compile
```
```console
cd CE2.0_SI_Solutions/W05/YoloV5/src/gst-plugin-mle
```
```console
mkdir build
```
```console
cd build
```

#### 6. Top Level directory structure of the application.
```console
   /root/DEVELOPMENT/gst-plugin-mle
   |
   |---CMakeLists.txt
   |---mle_engine
   |---mle_gst_snpe
   |---build
```

#### 7. Now you can perform the on-device compilation
```console
cmake -DCMAKE_VERBOSE_MAKEFILE=1 -DSNPE_ENABLE:BOOL=ON -DSNPE_SDK_BASE_DIR=<SNPE Directory path>/snpe-1.68.0.3932 -DFASTCV_SDK_DIR=<FastCV directory path> ..
```
```console
make
```
```console
make install
```
#### 8. Verify if qtimlesnpe plugin loading successfully
```console
gst-inspect-1.0 qtimlesnpe
```
Try Troubleshooting steps if it fails to load.

### Troubleshooting
##### 1. Delete the gstreamer cache
```console
rm -fr ~/.cache/gstreamer-1.0/registry.aarch64.bin
```

##### 2. Try again
1. Check if you are using same SDK for pushing and compiling the libraries.
2. Repeat the steps again

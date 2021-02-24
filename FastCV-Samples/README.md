## 1  Put the demo code at the right position
FastCV is a computer vision library being released by Qualcomm. The library is targeted toward very sophisticated Computer Vision (CV) application developers who are interested in writing CV applications that run on mobile devices. This demo will help you develop fastcv faster. For more info you can refer this website https://developer.qualcomm.com/software/fastcv-sdk.
## 2 Decompress the package

Depress the package, you will get three folders: fastcvArithmetic fastcvcolorconversion fastcvdownscale
```
$ git clone https://github.com/quic/sample-apps-for-Qualcomm-Robotics-RB5-platform.git
$ cd FastCV-Samples
```
## 3 Move code to right position
For this step, the Hexagon SDK should be installed. Download Hexagon DSP SDK from Qualcomm developer network https://developer.qualcomm.com/software/hexagon-dsp-sdk. Install any dependencies to setup the required environment for Hexagon SDK.


Decompress the compressed package, and place the demo code obtained from the decompression in Qualcomm / hexagon_ SDK / 3.4.2 / examples / fastcv / directory, execute the following command

// examples_name : fastcvArithmetic/fastcvcolorconversion/fastcvdownscale
```
# cd ~/Qualcomm/Hexagon_SDK/3.4.2 && source setup_sdk_env.source
# cd examples/fastcv/examples_name
# make tree V=UbuntuARM_Debug_aarch64
```

The excutable file will be generated on the UbuntuARM_Debug_aarch64/ship/
in order to excute the file, please push the file into rb5 board

Now using fastcvdownscale as example to demonstrate how to develop the fastcv. To get start, you need to prepare src.yuv (640x480,the resolution can be defined on the source code)as input.
//  You need to create workspace first
```
# adb shell
# mkdir -p /data/demofastcv/image
# exit
```
// Then you can run the demo in board
```
# adb push UbuntuARM_Debug_aarch64/ship/fastcvdownscale /data/demofastcv/
# adb push src.yuv /data/demofastcv/image/
# adb shell && cd /data/demofastcv/
# ./fastcvdownscale
```

After excute the file, dst file saved in /data/demofastcv/image/dst.y

```
# adb pull /data/demofastcv/image/dst.y ./
# adb pull /data/demofastcv/image/dst.yuv ./
```
//View the image generated after downscaling
```
#ffplay -f rawvideo -pix_fmt gray -s 320x240 dst.y         
```

## License
This is licensed under the BSD 3-clause-Clear “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.

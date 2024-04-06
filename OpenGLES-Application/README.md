# OpenGL ES Examples
# Overview
Each demo manages the window through the glfw frame. Using the API provided by OpenGLES and can be run and displayed in the Weston window.

## 1.  Init
### 1.1 Set opengles environment
```
$ adb shell
$ apt-get install extra-cmake-modules wayland-protocols libsdl2-dev libglm-dev
```
### 1.2 Download source code
```
$ cd /data
$ git clone https://github.com/quic/sample-apps-for-Qualcomm-Robotics-RB5-platform.git
$ cd OpenGLES-Application/
$ cp weston.sh /data
$ git clone https://github.com/dhustkoder/glfw.git
$ cp qualcomm.jpg glfw/examples/
$ cp 0001-Adapter-Opengles-API.patch 0001-Modify-opengles-demo.patch 0001-Add-opengles-demo.patch   glfw/
$ cd glfw/
$ git apply 0001-Adapter-Opengles-API.patch
$ git apply 0001-Modify-opengles-demo.patch
$ git apply 0001-Add-opengles-demo.patch
```
## 2. Build
```
$ mkdir build/
$ cd build/
$ cmake -DGLFW_USE_WAYLAND=ON ..
$ make -j8
```
## 3. Run
### 3.1 Execute weston script, run weston:
```
$ cd /data
$ sh weston.sh
```
### 3.2 Run demo
```
$ cd /data/OpenGLES-Application/glfw/build
// The offscreen demo Will not be displayed directly, but input the drawn triangle into offscreen.png
$ ./examples/offscreen
$ ./examples/sharing
$ ./examples/simple
$ ./examples/heightmap
$ ./examples/magic_box
```
// Note: Currently this app only supports offscreen, sharing, simple, heightmap, magic_box.
## License
This is licensed under the BSD 3-clause-Clear “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.

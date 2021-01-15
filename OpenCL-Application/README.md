# opencl simple code
# Overview
OpenCL-Sample-code shows OpenCL from three examples.
"FFT" shows the use of OpenCL for fast Fourier transform;
"Benchmark" shows the reading and writing rate when the memory unit is respectively Byte, KB, MB;
"Matrix_multiply" shows the multiplication of two 20*20 matrices,And print out the results of the two input matrices and the multiplication of the two matrices on the screen

## 1. Download code:
```
$ adb shell
$ cd home
$ git clone https://github.qualcomm.com/Robotics/sample-apps-for-Qualcomm-Robotics-RB5-platform.git
$ cd OpenCL-Application
```
## FFT
### 1.Init:
```
$ cd FFT
$ git clone https://github.com/bane9/OpenCL-FFT
$ cp 0001-OpenCL-Sample-App-FFT.patch ./OpenCL-FFT
$ cd OpenCL-FFT
$ git apply 0001-OpenCL-Sample-App-FFT.patch
```
## 2. compile
```
$ make
```
## 3. Run:
```
$ ./test
```
## Benchmark-OpenCL
### 1.Init:
```
$ cd Benchmark
$ git clone https://github.com/nerdralph/cl-mem.git
$ cp 0001-OpenCL-Sample-App-Benchmark.patch ./cl-mem
$ cd cl-mem
$ git apply 0001-OpenCL-Sample-App-Benchmark.patch
```
### 2. compile
```
$ make
```
### 3. Run:
```
./benchmark
```
## opencl simple code for matrix multiplication

### 1.Init:
```
$ cd matrix_multiply/
```
### 2. compile
```
$ make
```
### 3. Run:
```
$ ./test
```
## License
This is licensed under the BSD 3-claus-clear “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.

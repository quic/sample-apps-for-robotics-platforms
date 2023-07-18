# opencl simple code
# Overview
OpenCL-Sample-code shows OpenCL from three examples.
"FFT" shows the use of OpenCL for fast Fourier transform;
"Benchmark" shows the reading and writing rate when the memory unit is respectively Byte, KB, MB;
"Matrix_multiply" shows the multiplication of two 20x20 matrices,And print out the results of the two input matrices and the multiplication of the two matrices on the screen

## 1. Download code:
Install RB5 LU SDK and source the environment on PC
```
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


### 2. compile
```
$ make
```
### 3. Run:
```
$ adb disable-verity
$ adb reboot
$ adb wait-for-device root
### The above three steps only need to be operated once and will always be valid.

$ adb push FFT /data
$ adb shell
# cd /data/FFT/OpenCL-FFT
# chmod 777 test
# ./test
```
![image](https://github.qualcomm.com/storage/user/27989/files/9e34541c-d70b-4560-a2a6-9846a779ae5a)

## Benchmark-OpenCL
### 1.Init:
```
$ cd Benchmark
$ git clone https://github.com/nerdralph/cl-mem.git
$ cp 0001-OpenCL-Sample-App-Benchmark.patch ./cl-mem
$ cd cl-mem
$ git apply 0001-OpenCL-Sample-App-Benchmark.patch
Delete line 7 (CC = gcc) in Makefile
```

### 2. compile
```
$ make
```
### 3. Run:
```
$ adb push benchmark /data
$ adb shell
# cd data
# chmod 777 benchmark
# ./benchmark
```
![image](https://github.qualcomm.com/storage/user/27989/files/8678537a-9cd8-43f3-ab07-43eba428fb1e)

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
$ adb push matrix_multiply/ /data
$ adb shell
# chmod 777 /data/matrix_multiply/test
# cd data/matrix_multiply/
# ./test
```
![image](https://github.qualcomm.com/storage/user/27989/files/6d77c70d-ce33-41ed-befa-e9ec90468769)

## License
This is licensed under the BSD 3-claus-clear “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.

# RB5 platform User Guide

### 1. Set up the environment
Install RB5 LU SDK and source the environment
Download sample code to PC

### 2. Compile
```
$ cd GPIO-samples
$ mkdir -p bin
$ cd src
$ make
```

### 3. Push binary to device
```
$ cd ../bin
$ adb push qrb5165_platform /data
```

## 2. Execution and description of test procedure

You can either compile or execute the test program directly in the folder /data/Platform/bin/

   ###     1. LED light controlling

```shell
$ adb disable-verity
$ adb reboot
$ adb wait-for-device root
### The above three steps only need to be operated once and will always be valid.

adb shell
cd /data/
chmod +x ./qrb5165_platform
./qrb5165_platform -led green 255
```

The third parameter could be 0-255 (0 is to turn off the LED, other values control brightness)

   ###     2. Simple GPIO input/output

```shell
adb shell
cd /sys/class/gpio
echo 400 > export
cd /data
./qrb5165_platform -gpio out 1 0
./qrb5165_platform -gpio in 1
```



The second parameter could be out or in to control the GPIO input or output

The third parameter could be gpio number.The GPIO offset is 399. 400=offset+1.(GPIO port range:399-511)

The fourth parameter could be 1 or 0 to control the GPIO output (If GPIO is the input then there is no such parameter)

The value of the GPIO input or output is displayed in the console after execution.

   ###     3. Receive GPIO interrupt event

```shell
adb shell
cd /data
./qrb5165_platform -irq 100
```

The second parameter could be gpio number

The GPIO interrupt type is displayed in the console after execution

Press enter to exit the program

The snapshot after execution is shown below:

![image](https://github.qualcomm.com/storage/user/27989/files/d69e3dac-75ee-432b-974d-d78861894768)


   ###     4. Button event catching

```shell
adb shell
cd /data
./qrb5165_platform -button
```
Then you can press any button(on/off vol+ vol-) to test it

The event of the key is displayed in the console after execution

Press enter to exit the program

The snapshot after execution is shown below:

![image](https://github.qualcomm.com/storage/user/27989/files/77fafc58-bbc6-435e-9eed-78b85e1b1d1c)

   ###     5. Simple PWM output

```shell
adb shell
cd /data
./qrb5165_platform -pwm 0 1000 500
```

The third parameter could be period in ns

The fourth parameter could be high level time in ns, must be less than the period (The time ratio is the duty cycle)


###     6. Read the SoC thermal temperature

```shell
adb shell
cd /data
./qrb5165_platform -temp 0
```

The second parameter could be thermal_zone number

The SoC thermal temperature get from the node "/sys/class/thermal/thermal_zone%d/temp"

The snapshot after execution is shown below:

![image](https://github.qualcomm.com/storage/user/27989/files/cea0c2de-8cdd-4eb0-b009-869e9551b53b)
## License
This is licensed under the BSD 3-Clause-Clear “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.

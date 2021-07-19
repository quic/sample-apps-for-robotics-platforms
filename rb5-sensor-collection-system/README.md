# rb5-sensor-collection-system

<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->


- [1 Summary](#1-summary)
  - [1.1 Host Computer Requirements](#11-host-computer-requirements)
  - [1.2 RB5 Software Requirements](#12-rb5-software-requirements)
  - [1.3 RB5 Hardware Requirements](#13-rb5-harware-requirements)
  - [1.4 Sensors Summary](#14-sensors-summary)
- [2 Hardware Setup](#2-hardware-setup)
- [3 Software Quickstart](#3-software-quickstart)
  - [3.1 Build the Package](#31-build-the-package)
  - [3.2 Install Dependencies](#32-install-dependencies)
  - [3.3 Install the Package](#33-install-the-package)
  - [3.4 Running the Program](#34-running-the-program)
- [4 Status Output](#4-status-output)
  - [4.1 Successful Start](#41-successful-start)
- [5 Log File Format](#5-log-file-format)
  - [5.1 Time Format and Timezone](#51-time-format-and-timezone)
  - [5.2 Log File Location](#52-log-file-location)
  - [5.3 Log File Format](#53-log-file-format)
  - [5.3.1 cam_ir](#531-cam_ir)
  - [5.3.2 cam_thermal](#532-cam_thermal)
  - [5.3.3 cam_stereo](#533-cam_stereo)
  - [5.3.4 cam_tracking](#534-cam_tracking)
  - [5.3.5 chirp](#535-chirp)
  - [5.3.6 imu_42688](#536-imu_42688)
  - [5.3.7 pressure](#537-pressure)
  - [5.3.8 mag](#538-mag)
  - [5.3.9 temp](#539-temp)
  - [5.3.10 gps](#5310-gps)
  - [5.3.11 imu_4622x](#5311-imu_4622x)
- [6 Configuration File](#6-configuration-file)
  - [6.1 Config File Location](#61-config-file-location)
  - [6.2 Config File Description](#62-config-file-description)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

This project is used to build an installable software package that facilitates interfacing with and logging of various sensors integrated with an RB5 platform.

## 1 Summary

### 1.1 Host Computer Requirements

- Ubuntu 18.04
- adb

### 1.2 RB5 Software Requirements

- RB5 Main Board with QRB5165 SOM
  - Chipcode Release 9.1 
- Python 3.X and pip3

-Verify WiFi is configured for your local network connection, update the configuration file with the command shown below. Change the "ssid" and "psk" to match your network credentials for local WiFi.

```bash
vi data/misc/wifi/wpa_supplicant.conf

```

```bash
apt update
apt upgrade
apt install python3-pip
python3 -m pip install --upgrade pip
```
- The following python packages:

```bash
pip3 install opencv-python pynmea2 setproctitle pyserial matplotlib pandas
```

### 1.3 Hardware Requirements

- (1) Qualcomm Robotics RB5 Platform Mainboard with QRB5165 SOM
- (1) Qualcomm Robotics Navigation Mezzanine
- (1) Qualcomm Tracking Camera Flex Cable OV9282
- (1) Qualcomm Tracking Camera (OV9282)
- (2) GMSL Stereo Camera (LI-AR0231-gmsl2-cfm-176h-000)
- (2) GMSL Stereo Camera Cable (FAK-SMZSMZ-008 3M)
- (1) Varifocal IR Camera (IMX322)
- (1) FLIR Lepton Smart I/O Module (2077-PURETHERMAL-2)
- (1) Flir Lepton 2.5 Thermal Imaging Module (Lepton 2.5)
- (3) Sonar Sensor (Chirp T201)
- (3) Sonar Sensor (Chirp T101)
- (6) Chirp Sonar flex cables
- (1) Qualcomm RB5 Mainboard 5G Mezzanine
- (1) 5G Quectel Modem (RM502Q-AE)
- (1) Male USBC to Male USBA 3.0 cable (For Thermal Imaging Module)
- (1) 4 pin JST Male to Male USBA 3.0 cable ~1ft (For IR camera)
- (1) 12VDC, 2.5A 30watt AC/DC power supply w/ 4.8mmx1.7mm barrel plug OR equivalent battery
- (1) Sd card for RB5 Mainboard
- (1) Micro USB cable for debugging the RB5

### 1.4 Sensors Summary

| ID | Sensor Type     | Application  | Sensor PN                    | Interface   |
|----|-----------------|--------------|------------------------------|-------------|
| 1  | IR Camera       | cam_ir       | Varifocal IMX322             | USB         |
| 2  | Thermal Camera  | cam_thermal  | FLIR Lepton 2.5              | USB         |
| 3  | Stereo Cameras  | cam_stereo   | LI-AR0231-gmsl2-cfm-176h-000 | GMSL        |
| 3  | Tracking Camera | cam_tracking | OV9282                       | MIPI        |
| 5  | Chirp Sensor    | chirp        | Chirp T201 / 101             | I2C         |
| 5  | Chirp Sensor    | chirp        | Chirp T201 / T101            | I2C         |
| 6  | IMU             | imu_42688    | ICM 42688                    | SPI         |
| 7  | Pressure        | pressure     | TDK InvenSense ICP-10111     | I2C         |
| 8  | Magnetometer    | mag          | IST 8310                     | I2C         |
| 9  | Temperature     | temp         | TDK B57861S0103A039          | I2C         |
| 10 | GPS             | gps          | Neo-m8n                      | UART        |

## 2 Hardware Setup

### 2.1 Block Diagram

![High Level Block Diagram](/images/QC-RB5-Asset-Inspection-Block-Diagram.png)

### 2.2 Installation Diagram

![Drone Installation Diagram](/images/QC-RB5-Asset-Inspection-Drone-Installation.png)

### 2.3 Connecting Peripherals

1. Ensure 0V9282 is attached to the Navigation Mezzanine by way of Tracking Camera Flex OV9282. One end of the flex cable should go to CAM1 on the Navigation Mezzanine. Each end of the flex cable has a label corresponding to the correct connection

2. Gather and assemble the RB5 Mainboard, Navigation Mezzanine, TDK Mezzanine and 5G Mezzanine to form the board stackup.

3. Attach (6) flex cables to CH1 - Ch6 on the TDK Mezzanine board with Chirp sensors attached to each flex cable.

3. Plug (2) GMSL cables into each of the GMSL1 and GMSL2 ports on the RB5 Mainboard. Plug the other end of the GMSL cables into the stereo camera (LI-AR0231-gmsl2-cfm-176h-00).

4. Insert an SD card into the SD slot on the RBS mainboard.

5. Attach a debug cable to RB5 mainboard via micro USB debug terminal.

6. Attach a power supply to RB5 mainboard via 4.8mmX1.7mm barrel jack to power on RB5.

7. Attach the USB cameras to the RB5 mainboard. By default, the thermal camera must be plugged in before the IR camera so that the devices are properly named in the output log file. You can change the priority in configuration file located at /data/config

### 2.4 IR / Thermal Cameras

IR and thermal cameras are connected to the gimbal within an electronics enclosure and will connect to the main electronics enclosure via USB cables. IR camera will connect with the RB5 mainboard via (2) USB ports.

## 3 Software Quickstart

### 3.1 Build the Package

First clone the required git submodules in order to start building the package.
=======
First clone the repository in the directory of your choice on your host computer running Ubuntu 18.04.

```bash
git clone https://github.qualcomm.com/Robotics/sample-apps-for-Qualcomm-Robotics-RB5-platform.git
```

Navigate to directory that you just cloned above.
```bash
cd rb5-sensor-collection-system/
```

Run the following to create the debian package.

```bash
./make_package.sh
```

### 3.2 Install Dependencies

### 3.3 Install the Package

Note: If not already done, please (a) Attach a debug cable from your host computer running Ubuntu 18.04 to the RB5 mainboard via USB C connector debug terminal and (b) Attach a power supply to RB5 mainboard via 4.8mmX1.7mm barrel jack to power on RB5 before proceeding.

After the `rb5-scs_M.m.b.deb` is generated, you can run the following to install while connected with USB.

```bash
./install_on_rb5.sh
```

We now need to build the libsee sensors module. This will take place inside the rb5. adb into your device and build the libsee module. This requires cmake so begin by installing cmake.

### 3.4 Running the Program

After installing, to enable sensor data collecetion and logging based upon the configuration specified by the `/etc/rb5-scs.confg` file (described in section 6), run the following commands.

```bash
cd /data/rb5-scs/
./rb5-scs-start
```

Upon starting sucessfully, you will see the status output specified below.

## 4 Status Output

After starting the program, it will run for the specified duration in the configuration file.

### 4.1 Successful Start

```bash
[2021-02-18-11:23:56.001] rb5-scs: cam_ir:       success
[2021-02-18-11:23:56.001] rb5-scs: cam_thermal:  success
[2021-02-18-11:23:56.001] rb5-scs: cam_ir:       success
[2021-02-18-11:23:56.001] rb5-scs: cam_tracking: success
[2021-02-18-11:23:56.001] rb5-scs: chirp:        success
[2021-02-18-11:23:56.001] rb5-scs: imu_42688:    success
[2021-02-18-11:23:56.001] rb5-scs: pressure:     success
[2021-02-18-11:23:56.001] rb5-scs: mag:          success
[2021-02-18-11:23:56.001] rb5-scs: temp:         success
[2021-02-18-11:23:56.001] rb5-scs: gps:          success
[2021-02-18-11:23:56.001] rb5-scs: perf:         success
```

## 5 Log File Format

### 5.1 Time Format and Timezone

All times are saved in local time from the system clock.

The format will be in string form like `2021-02-18-11:23:56.000`, or in Linux epoch time (with millisceonds) like `1613753596.123`, supporting a resolution of 1kHz max.

### 5.2 Log File Location

- The default `log_root` is the SD card.
- At the start of each "session", a `session_start_time` directory is created in the `log_root`.
- For camera based sensors, timestamped (linux epoch time) pngs will be captured and saved
- For non-camera based sensors, a CSV log file will be created and logged to
- Log file location is at /tmp/rb5-scs/
=======
- To pull a log file, open a new terminal on your host computer.
- Log file location is at /tmp/rb5-scs/

```bash
adb pull /tmp/rb5-scs
```

- To pull logs locally, run adb pull /tmp/rb5-scs

| ID  | Sensor Type     | Application  | Log Location                                                        |
|-----|-----------------|--------------|---------------------------------------------------------------------|
| 1   | IR Camera       | cam_ir       | [log_root]/[session_start_time]/cam_ir/[linux_time_stamp].png       |
| 2   | Thermal Camera  | cam_thermal  | [log_root]/[session_start_time]/cam_thermal/[linux_time_stamp].png  |
| 3   | Stereo Cameras  | cam_stereo   | [log_root]/[session_start_time]/cam_thermal/[linux_time_stamp].png  |
| 3   | Tracking Camera | cam_tracking | [log_root]/[session_start_time]/cam_tracking/[linux_time_stamp].png |
| 5   | Chirp Sensor    | chirp        | [log_root]/[session_start_time]/chirp/log         |
| 6   | IMU             | imu_42688    | [log_root]/[session_start_time]/imu_42688/log     |
| 7   | Pressure        | pressure     | [log_root]/[session_start_time]/pressure/log      |
| 8   | Magnetometer    | mag          | [log_root]/[session_start_time]/mag/log           |
| 9   | Temperature     | temp         | [log_root]/[session_start_time]/temp/log          |
| 10  | GPS             | gps          | [log_root]/[session_start_time]/mag/log           |
| 100 | Performance     | perf         | [log_root]/[session_start_time]/perf/log          |

### 5.3 Log File Format

### 5.3.1 cam_ir

Linux epoch timestamped `.png` in `[log_root]/[session_start_time]/cam_ir/`

Usage:
Running ./rb5-log-uvc on it's own will take photos continuously every 5 seconds and log to /var/log/uvc/
-c <count> or --count <count> specifies the number of photos to be taken
-t <time> or --time <time> specifies the time between frames in SECONDS
-d <dir> or --dir <dir> specifies output directory, where PNGs will be placed

Daemon:
systemctl start uvc-log
default config file is at /etc/uvcconf.ini

### 5.3.2 cam_thermal

Linux epoch timestamped `.png` in `[log_root]/[session_start_time]/cam_thermal/`

### 5.3.3 cam_stereo

Linux epoch timestamped `.png` in `[log_root]/[session_start_time]/cam_stereo/`

### 5.3.4 cam_tracking

Linux epoch timestamped `.png` in `[log_root]/[session_start_time]/cam_tracking/`

### 5.3.5 chirp

A `log` CSV file located in `[log_root]/[session_start_time]/chirp/` with the following format:

```bash
timestamp, sensor ID, distance
```

Where:

- timestamp - floating point, epoch time
- sensor ID - int32, id=5
- distance - floating point, meters

### 5.3.6 imu_42688

A `log` CSV file located in `[log_root]/[session_start_time]/imu_42688/` with the following format:

```bash
Timestamp, DSP Timestamp, accelX, accelY, accelZ, gryoX, gryoY, gyroZ
```

Where:

- timestamp - floating point, epoch time
- DSP Timestamp - int64, SLPI Timestamp
- accelX - floating point, acceleration X axis
- accelY - floating point, acceleration Y axis
- accelZ - floating point, acceleration Z axis
- gyroX - floating point, gyro X axis
- gyroY - floating point, gyro Y axis
- gyroZ - floating point, gyro Z axis

### 5.3.7 pressure

A `log` CSV file located in `[log_root]/[session_start_time]/pressure/` with the following format:

```bash
timestamp, DSP timestamp, pressure
```

Where:

- timestamp - floating point, epoch time
- dsp timestamp - int64, SLPI Timestamp
- pressure - floating point, pressure, kPa

### 5.3.8 mag

A `log` CSV file located in `[log_root]/[session_start_time]/mag/` with the following format:

```bash
timestamp, magX, magY, magZ
```

Where:

- timestamp - floating point, epoch time
- magX - floating point, millitesla, mT
- magY - floating point, millitesla, mT
- magZ - floating point, millitesla, mT

### 5.3.9 temp

A `log` CSV file located in `[log_root]/[session_start_time]/temp/` with the following format:

```bash
timestamp, tempC
```

- timestamp - floating point, epoch time
- tempC - floating point, temperature, C

### 5.3.10 gps

A `log` CSV file located in `[log_root]/[session_start_time]/gps/` with the following format:

```bash
timestamp, lat, long, Speed Over Ground
```

- timestamp - floating point, epoch time
- lat - floating point, latitude
- long - floating point, longitude
- speed over ground - floating point, Knots

### 5.3.11 imu_4622x

A `log` CSV file located in `[log_root]/[session_start_time]/imu_4622x/` with the following format:

```bash
timestamp, sensor ID, accelX, accelY, accelZ, gryoX, gryoY, gyroZ
```

Where:

- timestamp - floating point, epoch time
- sensor ID - int32, id=11
- accelX - floating point, acceleration X axis
- accelY - floating point, acceleration Y axis
- accelZ - floating point, acceleration Z axis
- gyroX - floating point, gyro X axis
- gyroY - floating point, gyro Y axis
- gyroZ - floating point, gyro Z axis

### 5.3.12 perf

A `log` CSV file located in `[log_root]/[session_start_time]/perf/` with the following format:

```bash
timestamp, cpu_usage
```

Where:

- timestamp - floating point, epoch time
- cpu_usage - float,  percentage

## 6 Configuration File

### 6.1 Config File Location

The configuration file is located at `/etc/rb5-scs.config`

### 6.2 Config File Description

- `log_path` specifies location of the log root, as described in section 5
- `log_duration` specifies the length in seconds to log (0 to log until the process is killed). Only 300 seconds is recommended but log duration can be set higher.
- `simulation` set to true to use simulated data.
- `sensors` holds an array of possible sensors that can be adjusted, using.
- `enabled` to enable or disable the sensor and.
- `rate` which is the sample rate in Hz or FPS.
- `config_resolution` enables ability to modify resolution(0 uses default resolution, 1 enables uses width and height parameters).
- `width` set width of image capture, only works if config_resolution is set to 1.
- `height` set height of image, only works if config_resolution is set to 1.
- `priority` sets priority for order in which to plug in IR and thermal cameras. 1 would be plugged in first, 2 is plugged in second.
- `save-format` configure which save format you'd like camera data to be saved in
  - stereo camera can be saved as png, h264, or mp4
  - tracking camera can be saved as png, h264, or mp4
  - ir camera can be saved as png or jpg
  - thermal camera can be saved as png or jpg

For example:

```json
{
	"log_path": "/sdcard",
	"simulation": "false",
	"sensors": [
		{
			"name": "cam_ir",
			"enabled": "true",
			"rate": 30
		},
		{
			"name": "cam_stereo",
			"enabled": "true",
			"rate": 30
		},
		...
```

The log file is loaded at start time only and modification during runtime will not be adhered to.

## 6 Plotting

To disable/enable plotting, you can modify a file located here: /data/rb5-scs/rb5-scs.py
Comment out the following lines:

  - #time.sleep(config['log_duration'])
  - #create_plots(log_path, config['log_duration'])


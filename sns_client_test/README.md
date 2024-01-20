# sns_client_test
```
A sample app based on sns_clent_example
The purpose is helping users to learn how to implement sensor client to connect to see
```

# Get Start (On build machine)

## Connect build machine to board through USB-C
```
$ adb shell
sh-4.4# cd /data
sh-4.4# git clone https://github.com/quic/sample-apps-for-Qualcomm-Robotics-RB5-platform.git

```
## Compile app on baord
```
sh-4.4# cd sns_client_test/src
sh-4.4# make

```
## Test (on board)
On main board, make sure dip switch (DIP_SW_0) settings as below:

DIP switch number
          1   2    3     4     5    6
       --------------------------------
State | ON | ON | OFF | OFF | OFF | ON |
       --------------------------------

The sensor device on board:
ICM-4x6xx, the SUID : 12370169555311111083ull

sh-4.4# chmod +x sns_client_test 

  Get help informatioon about sns_client_test
sh-4.4# ./sns_client_test -h

  Run accel test for 10 senconds
sh-4.4# ./sns_client_test -s0 -t10
 
## License
This is licensed under the BSD 3-Clause-Clear “New” or “Revised” License. Check out the [LICENSE](LICENSE) for more details.

#!/bin/sh

SAVE_PATH="./"
LOG_PATH=$(grep log_path config/rb5-scs.config | awk '{print $2}' | sed 's/,//g' | sed 's/\"//g')
echo $LOG_PATH

adb pull $LOG_PATH $SAVE_PATH

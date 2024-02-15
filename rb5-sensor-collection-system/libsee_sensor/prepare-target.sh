#!/bin/bash

print_help() {
    echo "Usage: prepare-target.sh <board type>"
    echo "Valid board types are: rb5, m0051"
    echo "Example: ./prepare-target.sh m0051"
}

# Verify arguments
if [[ $# -eq 0 ]]; then
    echo "Error: Board type not specified"
    print_help
    exit -1
fi

if [[ $1 == "rb5" || $1 == "m0051" ]]; then
    BOARD_TYPE=$1
    echo "Board is $1"
else
    echo "Error: Invalid board type $1"
    print_help
    exit -1
fi

# Install the Qualcomm SEE sensors package
# TODO: Detect if has already been installed and skip all of this.
# TODO: Install in system image so that this isn't needed at all
SENSOR_SEE_PKG=sensors-see-qti-dev_0-r0_arm64.deb
if [[ ! -f $SENSOR_SEE_PKG ]]; then
    echo "Error, $SENSOR_SEE_PKG not found."
    echo "       Target not properly setup."
    echo "       See README.md for full instructions"
    exit -1
fi
dpkg --install $SENSOR_SEE_PKG

# Make sure cmake is installed
apt-get install -y cmake

# TODO: Figure out proper packages that have these include files or
#       add them to the system image build. We shouldn't have to do
#       this.
cp -r include/android/ /usr/include
cp -r include/google/ /usr/include
cp -r include/cutils/ /usr/include
cp -r include/utils/ /usr/include

# Setup the proper configuration for the board
cp config/qrb5165_icm4x6xx_0.json.m0051 /system/etc/sensors/config
cp config/qrb5165_icp101xx_0.json.m0051 /system/etc/sensors/config

cd /system/etc/sensors/config

mv qrb5165_icm4x6xx_0.json qrb5165_icm4x6xx_0.json.rb5
mv qrb5165_icp101xx_0.json qrb5165_icp101xx_0.json.rb5

rm -f qrb5165_icm4x6xx_0.json
rm -f qrb5165_icp101xx_0.json

if [[ $BOARD_TYPE == "rb5" ]]; then
    ln -s qrb5165_icm4x6xx_0.json.rb5 qrb5165_icm4x6xx_0.json
    ln -s qrb5165_icp101xx_0.json.rb5 qrb5165_icp101xx_0.json
else
    ln -s qrb5165_icm4x6xx_0.json.m0051 qrb5165_icm4x6xx_0.json
    ln -s qrb5165_icp101xx_0.json.m0051 qrb5165_icp101xx_0.json
fi

cd -

# TODO: Setup these links in the system image. Shouldn't have to do that here.
cd /usr/lib

if [[ ! -f libcutils.so ]]; then
    ln -s libcutils.so.0.0.0 libcutils.so
fi
if [[ ! -f liblog.so ]]; then
    ln -s liblog.so.0.0.0 liblog.so
fi
if [[ ! -f libprotobuf.so ]]; then
    ln -s libprotobuf.so.13.0.0 libprotobuf.so
fi

cd -

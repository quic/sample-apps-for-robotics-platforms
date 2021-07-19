#!/bin/sh

VERSION=$(grep "Version" ./DEBIAN/control | cut -d' ' -f 2)
PACKAGE=$(grep "Package" ./DEBIAN/control | cut -d' ' -f 2)
DEB_NAME=${PACKAGE}-${VERSION}.deb
PACKAGE_BUILD_DIR=${PACKAGE}-${VERSION}

rm -r $DEB_NAME
rm -r $PACKAGE_BUILD_DIR

mkdir -p $PACKAGE_BUILD_DIR/etc/
mkdir -p $PACKAGE_BUILD_DIR/data/
mkdir -p $PACKAGE_BUILD_DIR/usr/local/bin

cp -r DEBIAN $PACKAGE_BUILD_DIR/
cp config/rb5-scs.config $PACKAGE_BUILD_DIR/etc/
cp -r src $PACKAGE_BUILD_DIR/data/rb5-scs
cp tdk-thermistor-app.c $PACKAGE_BUILD_DIR/usr/local/bin
cp read_magnetometer.c $PACKAGE_BUILD_DIR/usr/local/bin

cp sensors-see-qti-dev_0-r0_arm64.deb libsee_sensor/
cp -r libsee_sensor $PACKAGE_BUILD_DIR/data/rb5-scs
cp sensors-see-qti-dev_0-r0_arm64.deb $PACKAGE_BUILD_DIR/data/rb5-scs/libsee_sensor

sudo chmod -R 0755 $PACKAGE_BUILD_DIR

dpkg -b $PACKAGE_BUILD_DIR

#!/bin/bash

# Fetch the needed packages from the bucket
# TODO: This can come from our Debian package repo in the future.
gsutil cp gs://qrb5165-sensor-libs/sensors-see-qti-dev_0-r0_arm64.deb .

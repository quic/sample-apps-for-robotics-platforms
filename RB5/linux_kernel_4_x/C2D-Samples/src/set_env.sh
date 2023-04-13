# Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

#!/bin/bash

# Copy msm_kgsl.h from the package you downloaded to the specified path.
tar -xvf libdrm-2.4.82.tar.bz2
cp libdrm-2.4.82/freedreno/kgsl/msm_kgsl.h /usr/include/linux
rm libdrm-2.4.82/ libdrm-2.4.82.tar.bz2 -rf

# Set up soft links.
ln -s /usr/lib/libion.so.0.0.0 /usr/lib/libion.so

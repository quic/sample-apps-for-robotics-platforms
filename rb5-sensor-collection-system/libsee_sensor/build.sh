#!/bin/bash
################################################################################
# Copyright (c) 2021 ModalAI, Inc. All rights reserved.
################################################################################

mkdir -p build
cd build
cmake ../
make -j4
make install
cd ../

# Copyright (c) 2020 Qualcomm Innovation Center, Inc.  All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

#! /bin/bash
amixer -c 0 cset name="MultiMedia1 Mixer TX_CDC_DMA_TX_3" "1"
amixer -c 0 cset name="TX DMIC MUX2" "DMIC1"
amixer -c 0 cset name="TX_CDC_DMA_TX_3 Channels" "One"
amixer -c 0 cset name="TX_AIF1_CAP Mixer DEC2" "1"
amixer -c 0 cset name='RX_MACRO RX0 MUX' 'AIF1_PB'
amixer -c 0 cset name='RX_CDC_DMA_RX_0 Channels' 'One'
amixer -c 0 cset name='RX INT0_1 MIX1 INP0' 'RX0'
amixer -c 0 cset name='RX INT0 DEM MUX' 'CLSH_DSM_OUT'
amixer -c 0 cset name='LO_RDAC Switch' 1
amixer -c 0 cset name='SpkrMono WSA_RDAC' 'Switch'
amixer -c 0 cset name='RX_CDC_DMA_RX_0 Audio Mixer MultiMedia1' 1

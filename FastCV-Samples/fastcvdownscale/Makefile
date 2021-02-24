# Copyright (c) 2020 Qualcomm Innovation Center, Inc. All Rights Reserved.
#
# SPDX-License-Identifier: BSD-3-Clause-Clear

# include the variant specific .min files
# V = hexagon --> hexagon.min
# V = android --> android.min
INCDIRS += \
   inc \
   
ifndef V
$(error Varaint must be provided, pass a variant by adding 'V=<desired varaint>' to your build command)
endif

V_TARGET = $(word 1,$(subst _, ,$(V)))

SUPPORTED_VS = $(default_VS)
include $(HEXAGON_SDK_ROOT)/build/make.d/$(V_TARGET)_vs.min
include $(HEXAGON_SDK_ROOT)/build/defines.min
include $(V_TARGET).min

# always last
include $(RULES_MIN)

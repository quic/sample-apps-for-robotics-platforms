/******************************************************************************
 *
 * Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of The Linux Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 *****************************************************************************/

#ifndef LE_PROP_OPS_H
#define LE_PROP_OPS_H

#include <stdbool.h>
#include <log/log.h>

#include "sys/system_properties.h"

/***********************************************************************
**  Type definitions
***********************************************************************/
#define MAX_ALLOWED_LINE_LEN        (PROP_NAME_MAX+PROP_VALUE_MAX+1)
#define MAX_PROPERTY_ITER           (2)
#define MAX_NUM_PROPERTIES          (10)

#define PROP_MSG_GETPROP '0'
#define PROP_MSG_SETPROP '1'

#define PROP_SERVICE_NAME "leprop-service"

#undef  LOG_TAG
#define LOG_TAG "leprop"

#undef LOG_DEBUG
//#define LOG_DEBUG

#undef LOG
#ifdef LOG_DEBUG
  #define LOG(fmt, args...) \
      ALOGD("%s:%d " fmt "\n", __func__, __LINE__, ##args)
#else
  #define LOG(fmt, args...) do {} while(0)
#endif

#define UNUSED(x) (void)(x)

/***********************************************************************
**  Exposed Functions
***********************************************************************/

/**
 * Get the property value for a given property name
 * @param Char * for property name
 * @param Char * for getting the property value from database/persist
 * @return True on success and false otherwise
 */
bool get_property_value(const char*, unsigned char *);

/**
 * Set the property value for a given property name
 * @param Char * for property name
 * @param Char * this is not used for set, used only for get
 * @return True on success and false otherwise
 */
bool set_property_value(const char*, unsigned char *);

#endif /* LE_PROP_OPS_H */

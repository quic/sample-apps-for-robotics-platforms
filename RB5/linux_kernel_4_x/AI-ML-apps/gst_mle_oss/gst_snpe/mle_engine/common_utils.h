/*
* Copyright (c) 2020, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
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
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <sstream>
#include <string>
#include <memory>
#include <cutils/properties.h>
#include <sys/mman.h>
#include <utils/Log.h>

#define MLE_LOGI(...) ALOGI("MLE: " __VA_ARGS__)
#define MLE_LOGE(...) ALOGE("MLE: " __VA_ARGS__)
#define MLE_LOGD(...) ALOGD("MLE: " __VA_ARGS__)

#define MLE_UNUSED(var) ((void)var)

#define DEFAULT_ALPHA 128

#define COLOR_TABLE_SIZE 32

typedef struct {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  uint8_t alpha;
} rgba;

static rgba color_table[COLOR_TABLE_SIZE] = {
  { //Black(background 0)
  .red   = 0,
  .green = 0,
  .blue  = 0,
  .alpha = DEFAULT_ALPHA
 },
 { //Maroon(aeroplane 1)
  .red   = 128,
  .green = 0,
  .blue  = 0,
  .alpha = DEFAULT_ALPHA
 },
 { //Green(bicycle 2)
  .red   = 0,
  .green = 128,
  .blue  = 0,
  .alpha = DEFAULT_ALPHA
 },
 { //Olive(bird 3)
  .red   = 128,
  .green = 128,
  .blue  = 0,
  .alpha = DEFAULT_ALPHA
 },
 { //Navy(boat 4)
  .red   = 0,
  .green = 0,
  .blue  = 128,
  .alpha = DEFAULT_ALPHA
 },
 { //Purple(bottle 5)
  .red   = 128,
  .green = 0,
  .blue  = 128,
  .alpha = DEFAULT_ALPHA
 },
 { //Teal(bus 6)
  .red   = 0,
  .green = 128,
  .blue  = 128,
  .alpha = DEFAULT_ALPHA
 },
 { //Silver(car 7)
  .red   = 192,
  .green = 192,
  .blue  = 192,
  .alpha = DEFAULT_ALPHA
 },
 { //Grey(cat 8)
  .red   = 128,
  .green = 128,
  .blue  = 128,
  .alpha = DEFAULT_ALPHA
 },
 { //Red(chair 9)
  .red   = 255,
  .green = 0,
  .blue  = 0,
  .alpha = DEFAULT_ALPHA
 },
 { //Lime(cow 10)
  .red   = 0,
  .green = 255,
  .blue  = 0,
  .alpha = DEFAULT_ALPHA
 },
 { //Yellow(diningtable 11)
  .red   = 255,
  .green = 255,
  .blue  = 0,
  .alpha = DEFAULT_ALPHA
 },
 { //Blue(dog 12)
  .red   = 0,
  .green = 0,
  .blue  = 255,
  .alpha = DEFAULT_ALPHA
 },
 { //Fuchsia(horse 13)
  .red   = 255,
  .green = 0,
  .blue  = 255,
  .alpha = DEFAULT_ALPHA
 },
 { //Aqua(motorbike 14)
  .red   = 0,
  .green = 255,
  .blue  = 255,
  .alpha = DEFAULT_ALPHA
 },
 { //White(person 15)
  .red   = 255,
  .green = 255,
  .blue  = 255,
  .alpha = DEFAULT_ALPHA
 },
 { //Honeydew2(potted plant 16)
  .red   = 215,
  .green = 255,
  .blue  = 215,
  .alpha = DEFAULT_ALPHA
 },
 { //Salmon1(sheep 17)
  .red   = 255,
  .green = 135,
  .blue  = 95,
  .alpha = DEFAULT_ALPHA
 },
 { //Orange1(sofa 18)
  .red   = 255,
  .green = 175,
  .blue  = 0,
  .alpha = DEFAULT_ALPHA
 },
 { //Gold1(train 19)
  .red   = 255,
  .green = 215,
  .blue  = 0,
  .alpha = DEFAULT_ALPHA
 },
 { //Thistle1(tv/monitor 20)
  .red   = 255,
  .green = 215,
  .blue  = 255,
  .alpha = DEFAULT_ALPHA
 },
 { //Cornsilk1(unknown 255)
  .red   = 255,
  .green = 255,
  .blue  = 215,
  .alpha = DEFAULT_ALPHA
 }
};
class Property {
 public:
  /** Get
   *    @property: property
   *    @default_value: default value
   *
   * Gets requested property value
   *
   * return: property value
   **/
  template <typename TProperty>
  static TProperty Get(std::string property, TProperty default_value) {
    MLE_UNUSED(color_table); // Consider removing
    TProperty value = default_value;
    char prop_val[PROPERTY_VALUE_MAX];
    std::stringstream s;
    s << default_value;
    property_get(property.c_str(), prop_val, s.str().c_str());

    std::stringstream output(prop_val);
    output >> value;
    return value;
  }

  /** Set
   *    @property: property
   *    @value: value
   *
   * Sets requested property value
   *
   * return: nothing
   **/
  template <typename TProperty>
  static void Set(std::string property, TProperty value) {
    std::stringstream s;
    s << value;
    std::string value_string = s.str();
    value_string.resize(PROPERTY_VALUE_MAX);
    property_set(property.c_str(), value_string.c_str());
  }
};

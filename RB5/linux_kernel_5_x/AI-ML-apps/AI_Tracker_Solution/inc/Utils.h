/* Copyright (c) 2021 Yifu Zhang

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Copyright (C) 2019 THL A29 Limited, a Tencent company. All rights reserved.

Licensed under the BSD 3-Clause License (the "License"); you may not use this file except
in compliance with the License. You may obtain a copy of the License at

https://opensource.org/licenses/BSD-3-Clause

Unless required by applicable law or agreed to in writing, software distributed
under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied. See the License for the
specific language governing permissions and limitations under the License. */

#ifndef UTILS_H_
#define UTILS_H_

#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <map>
#include <opencv2/imgproc.hpp>
#include <jsoncpp/json/json.h>
#include <unordered_map>

using namespace std;
using namespace cv;

using chrono::high_resolution_clock;
using chrono::duration_cast;
using chrono::duration;
using chrono::milliseconds;

#define QS_SUCCESS 0
#define QS_ERROR -1

#define PRINT(fmt, ...) { \
    printf(fmt, ##__VA_ARGS__); \
}

#define LOG(level, fmt, ...) { \
    PRINT("[%s] - %s: " fmt, #level, __func__, ##__VA_ARGS__); \
}

//#define DEBUG
#ifdef DEBUG
    #define LOG_DEBUG(fmt, ...)   LOG(DEBUG, fmt, ##__VA_ARGS__)
#else
    #define LOG_DEBUG(fmt, ...)  ((void)0)
#endif

#define LOG_INFO(fmt, ...) { \
    LOG(INFO, fmt, ##__VA_ARGS__); \
}

#define LOG_WARN(fmt, ...) { \
    LOG(WARN, fmt, ##__VA_ARGS__); \
}

#define LOG_ERROR(fmt, ...) { \
    LOG(ERROR, fmt, ##__VA_ARGS__); \
}

#define IMAGE_CHAN_SIZE_F32(width, height) ((width) * (height)*4)
#define RGB_IMAGE_SIZE_F32(width, height) ((width) * (height)*3 * 4)

// Inference hardware runtime.
typedef enum runtime {
    CPU = 0,
    GPU,
    GPU_FLOAT16,
    DSP,
    AIP
}runtime_t;

typedef enum PerformanceProfile {
    DEFAULT = 0,
    /// Run in a balanced mode.
    BALANCED = 0,
    /// Run in high performance mode
    HIGH_PERFORMANCE = 1,
    /// Run in a power sensitive mode, at the expense of performance.
    POWER_SAVER = 2,
    /// Use system settings.  SNPE makes no calls to any performance related APIs.
    SYSTEM_SETTINGS = 3,
    /// Run in sustained high performance mode
    SUSTAINED_HIGH_PERFORMANCE = 4,
    /// Run in burst mode
    BURST = 5,
    /// Run in lower clock than POWER_SAVER, at the expense of performance.
    LOW_POWER_SAVER = 6,
    /// Run in higher clock and provides better performance than POWER_SAVER.
    HIGH_POWER_SAVER = 7,
    /// Run in lower balanced mode
    LOW_BALANCED = 8,
}performance_t;

template <class T>
void ClearVector(std::vector<T>& vt) 
{
    std::vector<T> vtTemp; 
    vtTemp.swap(vt);
}

#endif
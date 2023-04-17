cmake_minimum_required(VERSION 3.8.2)
project(GST_PLUGIN_QTI_OSS_MLE
  VERSION 1.0
  LANGUAGES C CXX
)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(SYSROOT_INCDIR /usr/include)
set(SYSROOT_LIBDIR /usr/lib)
set(SYSROOT_AARCH_LIBDIR /usr/lib/aarch64-linux-gnu)

include_directories(${SYSROOT_INCDIR})
link_directories(${SYSROOT_LIBDIR} ${SYSROOT_AARCH_LIBDIR} ${SYSROOT_AARCH_LIBDIR}/gstreamer-1.0/)

# Generate configuration header file.
configure_file(config.h.in config.h @ONLY)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Precompiler definitions.
add_definitions(-DHAVE_CONFIG_H)
add_definitions(-DANDROID)
if (DELEGATE_SUPPORT)
	add_definitions(-DDELEGATE_SUPPORT)
endif()

# Common compiler flags.
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=maybe-uninitialized -Wno-error=unused-result -Wno-error=format= -Wformat -Wformat-security -Wall -Wextra -Werror -fPIC -MD -MT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-terminate")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-g -O2")

add_subdirectory(mle_engine)
add_subdirectory(mle_gst_tflite)
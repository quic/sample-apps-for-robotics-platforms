cmake_minimum_required(VERSION 3.8.2)

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE HOST_ARCHITECTURE )
if (HOST_ARCHITECTURE MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  message( STATUS "Building for aarch64 on ${HOST_ARCHITECTURE}")

  set(CMAKE_SYSTEM_NAME Linux)
  set(CMAKE_SYSTEM_PROCESSOR aarch64)

  set(CMAKE_FIND_ROOT_PATH /usr/aarch64-linux-gnu)

  set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
  set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

  set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
  set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
  set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
  set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
else()
  set(SYSROOT_INCDIR /usr/include)
  set(SYSROOT_LIBDIR /usr/lib)
  set(SYSROOT_AARCH_LIBDIR /usr/lib/aarch64-linux-gnu)
endif ()

project(GST_PLUGIN_QTI_OSS_MLE
  VERSION 1.0
  LANGUAGES C CXX
)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(SNPE_INCDIR ${SNPE_SDK_BASE_DIR}/include/zdl)
set(SNPE_LIBDIR ${SNPE_SDK_BASE_DIR}/lib/aarch64-ubuntu-gcc7.5)
set(GST_PLUGIN_INCDIR ${CMAKE_CURRENT_SOURCE_DIR}/../include)

include_directories(${SYSROOT_INCDIR} ${FASTCV_SDK_DIR} ${SNPE_INCDIR})
link_directories(${SYSROOT_LIBDIR} ${SYSROOT_AARCH_LIBDIR} ${SNPE_LIBDIR})

# Generate configuration header file.
configure_file(config.h.in config.h @ONLY)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${GST_PLUGIN_INCDIR})

# Precompiler definitions.
add_definitions(-DHAVE_CONFIG_H)
if (DELEGATE_SUPPORT)
add_definitions(-DDELEGATE_SUPPORT)
endif()

find_package(PkgConfig)
pkg_search_module(GLIB REQUIRED glib-2.0)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_ALLOC REQUIRED gstreamer-allocators-1.0)
pkg_check_modules(GST_VIDEO REQUIRED gstreamer-video-1.0)

# Common compiler flags.
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=maybe-uninitialized -Wno-error=unused-result -Wno-error=format= -Wformat -Wformat-security -Wall -Wextra -Werror -fPIC -MD -MT")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-terminate")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-g -O2")

add_subdirectory(mle_engine)
add_subdirectory(mle_gst_snpe)



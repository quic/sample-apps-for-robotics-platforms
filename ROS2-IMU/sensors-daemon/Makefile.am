# Makefile.am - Automake script for sns_imud_ros

AM_CPPFLAGS = -Werror                           \
              -Wall                             \
              -Wno-unused-parameter             \
              -Wno-unused-variable              \
              -fexceptions                      \
              -DSNS_TARGET_LE_PLATFORM          \
              -I../sensors-daemon/inc                       \
              -I${WORKSPACE}/vendor/qcom/proprietary/sensors-see/sensors-log               \
              -I${WORKSPACE}/vendor/qcom/proprietary/sensors-see/sensors-diag-log/inc      \
              -I./recipe-sysroot/usr/include   \
              -I${WORKSPACE}/qmi-framework/inc

requiredlibs = ../recipe-sysroot/usr/lib/libsnsapi.so       \
               ../recipe-sysroot/usr/lib/libsensorslog.so   \
               -llog                              \
               -lcutils                           \
               $(QMIFRAMEWORK_LIBS)               \
               $(PROTOBUF_LIBS)

cpp_sources = src/sns_client_example.cpp        \
              src/ssc_connection_reference.cpp  \
              src/ssc_suid_util.cpp

bin_PROGRAMS =  sns_imud_ros
sns_imud_ros_SOURCES = $(cpp_sources)
sns_imud_ros_CC = @CC@
sns_imud_ros_CPPFLAGS = $(AM_CPPFLAGS)
#if USE_GLIB
#sns_client_example_cpp_CPPFLAGS += -DUSE_GLIB @GLIB_CFLAGS@
#sns_client_example_cpp_LDFLAGS = @GLIB_LIBS@
#endif
sns_imud_ros_LDADD = $(requiredlibs)

lib_LTLIBRARIES = libsns_imud_ros.la
libsns_imud_ros_la_CC = @CC@
libsns_imud_ros_la_SOURCES = $(cpp_sources)
libsns_imud_ros_la_CPPFLAGS = $(AM_CPPFLAGS)
libsns_imud_ros_la_LDFLAGS = -shared -version-info 1:0:0
libsns_imud_ros_la_LIBADD = $(requiredlibs)
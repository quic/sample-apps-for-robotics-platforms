# Makefile.am - Automake script for sensors client

AM_CPPFLAGS = -Werror                           \
              -Wall                             \
              -Wno-unused-parameter             \
              -Wno-unused-variable              \
              -fexceptions                      \
              -I../sensors-client/inc           \
              -I$(INC_ROOT)/usr/include/glib-2.0 \
              -Dstrlcpy=g_strlcpy -Dstrlcat=g_strlcat

requiredlibs = -lglib-2.0

cpp_sources = src/imu_client.cpp

cpp_test_sources = src/imu_client.cpp    \
                   src/imu_client_test.cpp

bin_PROGRAMS =  imu_client_test
imu_client_test_SOURCES = $(cpp_test_sources)
imu_client_test_CC = @CC@
imu_client_test_CPPFLAGS = $(AM_CPPFLAGS)
imu_client_test_LDADD = $(requiredlibs)

lib_LTLIBRARIES = libimu_client.la
libimu_client_la_CC = @CC@
libimu_client_la_SOURCES = $(cpp_sources)
libimu_client_la_CPPFLAGS = $(AM_CPPFLAGS)
libimu_client_la_LDFLAGS = -shared -version-info 1:0:0
libimu_client_la_LIBADD = $(requiredlibs)
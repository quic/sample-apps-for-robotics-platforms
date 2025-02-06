cmake_minimum_required(VERSION 3.10.0)
project(libsee_sensor VERSION 1.0.0 DESCRIPTION "SEE sensor library")

# TODO: Add proper C and C++ build flags

add_library(see_sensor SHARED
            src/see_sensor_common.cpp
            src/see_sensor_icm4x6xx.cpp
            src/see_sensor_icp101xx.cpp)

set_target_properties(see_sensor PROPERTIES
                      VERSION ${PROJECT_VERSION}
                      SOVERSION 1
                      PUBLIC_HEADER api/see_sensor.h)

include_directories(api/)

add_executable(test_see_sensor_icm4x6xx
               src/test_see_sensor_icm4x6xx.c)

add_executable(test_see_sensor_icp101xx
               src/test_see_sensor_icp101xx.c)

add_executable(log_sensor_icm4x6xx
               src/log_sensor_icm4x6xx.c)

add_executable(log_sensor_icp101xxx
               src/log_sensor_icp101xxx.c)

target_link_libraries(see_sensor
                      ssc
                      protobuf
                      sensorslog
                      snsapi
                      snsdiaglog
                      # glib-2.0
                      dl
                      log
                      cutils
                      pthread)

target_link_libraries(test_see_sensor_icm4x6xx
                      see_sensor)

target_link_libraries(test_see_sensor_icp101xx
                      see_sensor)

target_link_libraries(log_sensor_icm4x6xx
                      see_sensor)

target_link_libraries(log_sensor_icp101xxx
                      see_sensor)

install(TARGETS see_sensor
        LIBRARY DESTINATION /usr/lib
        PUBLIC_HEADER DESTINATION /usr/include)

install(TARGETS log_sensor_icm4x6xx
        DESTINATION /usr/bin)

install(TARGETS log_sensor_icp101xxx
        DESTINATION /usr/bin)

install(TARGETS test_see_sensor_icm4x6xx
        DESTINATION /usr/bin)

install(TARGETS test_see_sensor_icp101xx
        DESTINATION /usr/bin)

cmake_minimum_required(VERSION 3.5.1)


# Compile options
add_compile_options(-std=c++11)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  "../out")
set(CMAKE_CXX_FLAGS_DEBUG "-fPIC -O0 -g -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-fPIC -O2 -Wall")

message(STATUS "source file path" ${PROJECT_SRC_ROOT})

file(GLOB_RECURSE SRC_FILE
    ../src/*.cpp
)

set(SNPE_INCDIR ${SNPE_SDK_BASE_DIR}/include/zdl)

set(SOURCE_FILE
    ${SRC_FILE}
)

add_executable(main ${SRC_FILE})
target_compile_options(main PUBLIC -fPIC  -O0 -g -Wall -Wnon-virtual-dtor)

# Header path
include_directories(
    "../inc"
    "/usr/include/glib-2.0"
    "/usr/lib/aarch64-linux-gnu/glib-2.0/include"
    "/usr/include/gstreamer-1.0"
    "/usr/local/include/opencv4"
    ${OpenCV_INCLUDE_DIRS}
     ${SNPE_INCDIR}
    ${JSON_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
)

message(${JSON_INCLUDE_DIRS})
message(${JSONCPP_INCLUDE_DIRS})

target_link_libraries(main PUBLIC pthread dl ${OpenCV_LIBS} ${GST_APP_LIBRARIES} ${JSON_LIBRARIES} jsoncpp SNPE jsoncpp)

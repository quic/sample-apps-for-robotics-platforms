cmake_minimum_required(VERSION 3.8.2)

project(imu-ros2node
        LANGUAGES C CXX)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories (./include)

if (CMAKE_CROSSCOMPILING)
  set(ament_cmake_core_DIR "${FOXY_DIR}/share/ament_cmake_core/cmake")
  include (${FOXY_DIR}/share/ament_cmake_core/cmake/ament_cmake_coreConfig.cmake)
  include_directories (${SYSROOT_INCDIR})
  include_directories (${SYSROOT_INCDIR}/Poco)
  include_directories (${FOXY_DIR}/include)
  link_directories(${SYSROOT_LIBDIR})
  link_directories(${FOXY_DIR}/lib)
else ()
  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
endif ()

add_executable(imu-ros2node src/imu_node.cpp)
add_executable(imu-ros2test src/imu_node_test.cpp)

if (CMAKE_CROSSCOMPILING)
  set(ROS-FOXY
    rclcpp
    rcutils
    yaml
    rcl
    rcl_interfaces__rosidl_generator_c
    rcl_interfaces__rosidl_typesupport_c
    rcl_interfaces__rosidl_typesupport_cpp
    rcl_logging_noop
    rcl_yaml_param_parser
    rosgraph_msgs__rosidl_typesupport_cpp
    message_filters
    ament_index_cpp
    class_loader
    rmw
    rmw_implementation
    rosidl_generator_c
    rosidl_typesupport_c
    rosidl_typesupport_cpp
    std_msgs__rosidl_generator_c
    std_msgs__rosidl_typesupport_c
    std_msgs__rosidl_typesupport_cpp
    sensor_msgs__rosidl_typesupport_cpp
    console_bridge
    builtin_interfaces__rosidl_generator_c
  )
  target_link_libraries(imu-ros2node PRIVATE
    pthread
    imu_client
    PocoFoundation
    ${ROS-FOXY}
  )
  target_link_libraries(imu-ros2test PRIVATE
    imu_client
    PocoFoundation
    ${ROS-FOXY}
  )
  install(TARGETS imu-ros2node imu-ros2test
    RUNTIME DESTINATION ${INSTALL_LIBDIR}/${PROJECT_NAME}
  )
else ()
  ament_target_dependencies(imu-ros2node
    rclcpp
    sensor_msgs
    geometry_msgs
  )
  target_link_libraries(imu-ros2node
    pthread
    imu_client
  )
  ament_target_dependencies(imu-ros2test
    rclcpp
    sensor_msgs
    geometry_msgs
  )
  install(TARGETS imu-ros2node imu-ros2test
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

if (NOT CMAKE_CROSSCOMPILING)
  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

ament_package()

cmake_minimum_required(VERSION 3.20)
project(transponder2ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find GeographicLib
# Append to CMAKE_MODULE_PATH since debian/ubuntu installs
# FindGeographicLib.cmake in a nonstand location
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(transponder_msgs REQUIRED)
find_package(GeographicLib REQUIRED)

# git hash and branch
execute_process(
    COMMAND git describe --abbrev=7 --dirty --always --tags
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
    COMMAND git rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    
add_definitions(-DGIT_INFO="${GIT_BRANCH} - ${GIT_VERSION}")

# Includes
include_directories(
    include
)

# Add executables
add_executable(transponder2ros_node src/transponder2ros_node.cpp
    src/transponder2ros/functions_ros.cpp
    src/transponder2ros/functions_serial.cpp
    src/transponder2ros/functions_udp.cpp
    src/transponder2ros/transponder2ros.cpp
    )
    target_link_libraries(transponder2ros_node
        ${rclcpp_TARGETS}
        ${std_msgs_TARGETS}
        ${transponder_msgs_TARGETS}
    )
    install(TARGETS
      transponder2ros_node
        DESTINATION lib/${PROJECT_NAME}
    )

add_executable(transponder2NavSatFix_node src/transponder2NavSatFix_node.cpp
    src/transponder2NavSatFix_node.cpp
    )
    target_link_libraries(transponder2NavSatFix_node
        ${rclcpp_TARGETS}
        ${std_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${transponder_msgs_TARGETS}
    )
    install(TARGETS
        transponder2NavSatFix_node
        DESTINATION lib/${PROJECT_NAME}
    )

add_executable(debug_node src/debug_node.cpp)
    target_link_libraries(debug_node
        ${rclcpp_TARGETS}
        ${std_msgs_TARGETS}
        ${transponder_msgs_TARGETS}
    )
    install(TARGETS
      debug_node
        DESTINATION lib/${PROJECT_NAME}
    )

add_executable(odom2transponder_node src/odom2transponder_node.cpp)
    # GeographicLib's Debian find module exports no CMake target, only libraries and include dirs.
    target_link_libraries(odom2transponder_node
        ${rclcpp_TARGETS}
        ${std_msgs_TARGETS}
        ${nav_msgs_TARGETS}
        ${geographic_msgs_TARGETS}
        ${transponder_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${GeographicLib_LIBRARIES}
    )
    target_include_directories(odom2transponder_node PRIVATE ${GeographicLib_INCLUDE_DIRS})
    install(TARGETS
    odom2transponder_node
        DESTINATION lib/${PROJECT_NAME}
    )

# Copy over config and launch files
install(DIRECTORY
  # config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/opt/ros/noetic/include".split(';') if "${prefix}/include;/opt/ros/noetic/include" != "" else []
PROJECT_CATKIN_DEPENDS = "controller_interface;dynamic_reconfigure;franka_hw;geometry_msgs;franka_core_msgs;hardware_interface;message_runtime;pluginlib;realtime_tools;roscpp".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lfranka_ros_controllers;/opt/ros/noetic/lib/aarch64-linux-gnu/libfranka.so.0.9.2".split(';') if "-lfranka_ros_controllers;/opt/ros/noetic/lib/aarch64-linux-gnu/libfranka.so.0.9.2" != "" else []
PROJECT_NAME = "franka_ros_controllers"
PROJECT_SPACE_DIR = "/home/student/meam520_ws/install_isolated"
PROJECT_VERSION = "0.7.1"

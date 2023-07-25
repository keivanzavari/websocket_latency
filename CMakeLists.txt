cmake_minimum_required(VERSION 3.1.0)
project(websocket_latency)

## Find catkin macros and libraries
find_package(
  catkin
  REQUIRED
  COMPONENTS rospy
             dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(cfg/DynamicParameters.cfg)

# Runtime dependencies
catkin_package(
  CATKIN_DEPENDS
  rospy
  dynamic_reconfigure
)

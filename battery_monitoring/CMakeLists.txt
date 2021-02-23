cmake_minimum_required(VERSION 2.8.3)
project(battery_monitoring)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  std_srvs
  message_generation
  sensor_msgs
  cmr_msgs
  sensor_msgs
  cmake_modules
  neo_msgs
)
#catkin_python_setup()
add_message_files(
   FILES
   BatteryMonitoring.msg
 )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(battery_monitoring
  src/battery_monitoring.cpp
  include/battery_monitoring/battery_monitoring.h
)

target_link_libraries(battery_monitoring
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############


install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

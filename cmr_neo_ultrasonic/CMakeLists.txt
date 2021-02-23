cmake_minimum_required(VERSION 2.8.3)
project(cmr_neo_ultrasonic)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  cmr_msgs
  message_generation
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(us_avoid_collision
  src/us_avoid_collision.cpp
  include/cmr_neo_ultrasonic/us_avoid_collision.h
  include/cmr_neo_ultrasonic/collision_dist.h
)

#add_dependencies(map_modifier ${PROJECT_NAME}_gencfg)

target_link_libraries(us_avoid_collision
  ${catkin_LIBRARIES}
)

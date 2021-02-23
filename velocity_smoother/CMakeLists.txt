cmake_minimum_required(VERSION 2.8.3)
project(velocity_smoother)

find_package(catkin REQUIRED COMPONENTS
dynamic_reconfigure
geometry_msgs
nav_msgs
roscpp
roslint
std_msgs
sensor_msgs
message_filters
spencer_tracking_msgs)

find_package(Boost REQUIRED thread)

generate_dynamic_reconfigure_options(cfg/params.cfg)

if(CMAKE_COMPILER_IS_GNUCXX)
  add_definitions(-std=gnu++0x)
else()
  add_definitions(-std=c++0x)
endif()

catkin_package()

### BUILD ###
include_directories(include include/cob_base_velocity_smoother ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})


add_executable(velocity_smoother src/cob_base_velocity_smoother/velocity_smoother.cpp src/velocity_smoother_node.cpp src/footprint.cpp)
add_dependencies(velocity_smoother ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity_smoother ${catkin_LIBRARIES} ${BOOST_LIBRARIES})

roslint_cpp()

### INSTALL ###
#install(TARGETS velocity_smoother
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#install(DIRECTORY config
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

cmake_minimum_required(VERSION 2.8.6)

project(rtt_ati_sensor)

find_package(catkin REQUIRED
  COMPONENTS 
  rtt_ros 
  rtt_roscomm
  std_msgs
  ati_sensor
  geometry_msgs
  cmake_modules)

#find_package(orocos_kdl REQUIRED)
#find_package(Eigen REQUIRED)

add_definitions(-DRTTCOMPONENT)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS})

orocos_component(rtt_ati_sensor
  src/rtt_ft_sensor.cpp
  )

set(COMPONENT_LIBS
  ${orocos_kdl_LIBRARIES}
  ${catkin_LIBRARIES})

target_link_libraries(${PROJECT_NAME} ${COMPONENT_LIBS})

orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS ${PROJECT_NAME}
  DEPENDS_TARGETS rtt_roscomm)
  
#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
  )
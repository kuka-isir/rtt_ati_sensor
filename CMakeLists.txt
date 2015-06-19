cmake_minimum_required(VERSION 2.8.6)

project(rtt_ati_sensor)

set(CMAKE_BUILD_TYPE Release)

SET(MAJOR_VERSION "1")
SET(MINOR_VERSION "0")
SET(PATCH_VERSION "0")
set(RTT_ATI_SENSOR_VERSION
  ${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION})

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

find_package(Boost 1.53 REQUIRED COMPONENTS atomic)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  )

orocos_component(${PROJECT_NAME}
  src/rtt_ft_sensor.cpp
  )

set(COMPONENT_LIBS
  ${orocos_kdl_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

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
  
  
###################
## Documentation ##
###################


# add a target to generate API documentation with Doxygen
if(BUILD_DOCUMENTATION)
find_package(Doxygen)
if(DOXYGEN_FOUND)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile @ONLY)
        execute_process(
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc
        #COMMENT "Generating API documentation with Doxygen" VERBATIM
        #DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile
        )
else()
message(FATAL_ERROR
      "Doxygen is needed to build the documentation. Please install it correctly")
endif()
endif()

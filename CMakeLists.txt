cmake_minimum_required(VERSION 2.8.3)
project(helelani_onboard)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES helelani_onboard
  CATKIN_DEPENDS roscpp std_msgs helelani_common
)

###########
## Build ##
###########

set(WORKSPACE_INCLUDES "")
foreach(f ${CATKIN_WORKSPACES})
  list(APPEND WORKSPACE_INCLUDES "${f}/include")
endforeach(f)
include_directories(src
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${WORKSPACE_INCLUDES})

add_executable(${PROJECT_NAME}_telemetry_node
               src/TelemetryNode.cpp
               src/DlnFinders.cpp src/DlnFinders.h
               src/Hmc5883lDriver.cpp src/Hmc5883lDriver.h
               src/AdxlDriver.cpp src/AdxlDriver.h
               src/Bmp085Driver.cpp src/Bmp085Driver.h
               src/i2c.cpp src/i2c.h
               src/analog.cpp src/analog.h
               src/minmea.cpp src/minmea.h
               src/GPSReader.cpp src/GPSReader.h)

add_executable(${PROJECT_NAME}_navigation_node
              src/NavigationNode.cpp)

## Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_telemetry_node PROPERTIES OUTPUT_NAME telemetry_node PREFIX "")
set_target_properties(${PROJECT_NAME}_navigation_node PROPERTIES OUTPUT_NAME navigation_node PREFIX "")

## Add cmake target dependencies of the executables
add_dependencies(${PROJECT_NAME}_telemetry_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_navigation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_telemetry_node ${catkin_LIBRARIES} udev)
target_link_libraries(${PROJECT_NAME}_navigation_node ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS
  ${PROJECT_NAME}_telemetry_node
  ${PROJECT_NAME}_navigation_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

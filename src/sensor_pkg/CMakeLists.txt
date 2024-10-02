cmake_minimum_required(VERSION 3.0.2)
project(sensor_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

## Find system dependencies
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /home/kyota/Downloads/MVS_STD_GML_V2.1.2_221208/MVS-2.1.2_x86_64_20221208/MVS/include
)

## Declare a C++ executable
add_executable(sensor_node src/camera_sensor.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(sensor_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  /home/kyota/Downloads/MVS_STD_GML_V2.1.2_221208/MVS-2.1.2_x86_64_20221208/MVS/lib/64/libFormatConversion.so
  /home/kyota/Downloads/MVS_STD_GML_V2.1.2_221208/MVS-2.1.2_x86_64_20221208/MVS/lib/64/libMediaProcess.so
  /home/kyota/Downloads/MVS_STD_GML_V2.1.2_221208/MVS-2.1.2_x86_64_20221208/MVS/lib/64/libMvCameraControl.so
  /home/kyota/Downloads/MVS_STD_GML_V2.1.2_221208/MVS-2.1.2_x86_64_20221208/MVS/lib/64/libMvCameraControlWrapper.so
  /home/kyota/Downloads/MVS_STD_GML_V2.1.2_221208/MVS-2.1.2_x86_64_20221208/MVS/lib/64/libMVGigEVisionSDK.so
  /home/kyota/Downloads/MVS_STD_GML_V2.1.2_221208/MVS-2.1.2_x86_64_20221208/MVS/lib/64/libMVRender.so
  /home/kyota/Downloads/MVS_STD_GML_V2.1.2_221208/MVS-2.1.2_x86_64_20221208/MVS/lib/64/libMvUsb3vTL.so
  )

#############
## Install ##
#############

# Mark executables for installation
install(TARGETS sensor_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sensor_pkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

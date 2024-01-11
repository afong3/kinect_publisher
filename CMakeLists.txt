cmake_minimum_required(VERSION 2.8.3)

project(kinect_publisher)

set(CMAKE_CXX_FLAGS "-std=c++11")
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS sensor_msgs image_transport roscpp rospy std_msgs genmsg)

include_directories(${catkin_INCLUDE_DIRS})
## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

add_executable(kinect_publisher src/kinect_publisher.cpp)
target_link_libraries(kinect_publisher ${catkin_LIBRARIES})
add_dependencies(kinect_publisher kinect_publisher_generate_messages_cpp)

## Libfreenect2 stuff

find_package(freenect2 REQUIRED)
find_package(PCL 1.3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(
    "/usr/include/libusb-1.0/"
    ${freenect2_INCLUDE_DIR}
    )

## PCL 

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(kinect_publisher 
    ${freenect2_LIBRARIES}
    ${PCL_LIBRARIES}
    )


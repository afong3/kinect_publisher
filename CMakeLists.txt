cmake_minimum_required(VERSION 2.8.3)

project(kinect_publisher)

set(CMAKE_CXX_FLAGS "-std=c++11")
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  message_filters 
  sensor_msgs 
  image_transport 
  roscpp 
  rospy 
  std_msgs 
  genmsg 
  cv_bridge 
  message_generation
  geometry_msgs)

add_message_files(
  FILES
  ObjectArray.msg
  )

## Generate added messages and services
generate_messages(
  DEPENDENCIES 
  std_msgs
  geometry_msgs
  )
  
  ## Declare a catkin package
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(kinect_publisher src/kinect_publisher.cpp)
target_link_libraries(kinect_publisher ${catkin_LIBRARIES})
add_dependencies(kinect_publisher kinect_publisher_generate_messages_cpp)

add_executable(image_writer src/image_writer.cpp)
target_link_libraries(image_writer ${catkin_LIBRARIES})
add_dependencies(image_writer image_writer_generate_messages_cpp)

add_executable(rgb_depth_publisher src/rgb_depth_publisher.cpp)
target_link_libraries(rgb_depth_publisher ${catkin_LIBRARIES})
add_dependencies(rgb_depth_publisher rgb_depth_publisher_generate_messages_cpp)

add_executable(registration src/registration.cpp)
target_link_libraries(registration ${catkin_LIBRARIES})
add_dependencies(registration registration_generate_messages_cpp)

## Libfreenect2 stuff

find_package(freenect2 REQUIRED)
find_package(PCL 1.3 REQUIRED)
find_package(OpenCV)

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

target_link_libraries(image_writer
    ${freenect2_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    )

target_link_libraries(rgb_depth_publisher
    ${freenect2_LIBRARIES}
    ${OpenCV_LIBS}
)

target_link_libraries(registration
   ${freenect2_LIBRARIES}
   ${OpenCV_LIBS}  
   ${PCL_LIBRARIES}
)

catkin_install_python(
  PROGRAMS 
  scripts/object_classifier.py
  scripts/object_classifier_roboflow.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

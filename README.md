# kinect_publisher

Circumventing the errors involved with running iai_kinect2 on ROS Noetic. This provides a couple of ways to get color and depth images from the KinectV2. Using kinect_publisher will publish the registered point cloud while rgb_depth_publisher + registration combined will produce the same result. Using the latter is important if any image processing needs to be done prior to registering the color onto the depth image


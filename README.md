# kinect_publisher

Circumventing the errors involved with running iai_kinect2 on ROS Noetic. This provides a couple of ways to get color and depth images from the KinectV2. Using kinect_publisher will publish the registered point cloud while rgb_depth_publisher + registration combined will produce the same result. Using the latter is important if any image processing needs to be done prior to registering the color onto the depth image

## Object Segmentation and Classification Pipeline
![Prototype Presentation](https://github.com/user-attachments/assets/d7add8f4-98eb-4a75-aab2-694995c4de4c)

![Prototype Presentation (1)](https://github.com/user-attachments/assets/1865f425-cdf9-41c9-9454-c9824bc7235a)

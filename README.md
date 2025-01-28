# kinect_publisher

Circumventing the errors involved with running iai_kinect2 on ROS Noetic. This provides a couple of ways to get color and depth images from the KinectV2. Using kinect_publisher will publish the registered point cloud while rgb_depth_publisher + registration combined will produce the same result. Using the latter is important if any image processing needs to be done prior to registering the color onto the depth image

## Object Segmentation and Classification Pipeline
![Final Presentation (1)](https://github.com/user-attachments/assets/709a6caf-adfe-4dff-bf72-097de2b2243d)
![Final Presentation (2)](https://github.com/user-attachments/assets/b009ba11-1663-4312-bc17-b4805da2656e)
![Final Presentation (3)](https://github.com/user-attachments/assets/d304760e-1f03-4b54-b11a-749e5c7c1ae2)
![Final Presentation (4)](https://github.com/user-attachments/assets/8e75d0d3-6cd3-4126-8057-88d58e932f36)
![Final Presentation](https://github.com/user-attachments/assets/24ad9e6d-da5a-46a6-b0b4-2c3d24b46c5a)

// ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
//#include <opencv2/highgui/highgui.hpp>
// #include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sstream> // for converting the command line parameter to integer

// libfreenect2 includes
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <fstream>
#include <cstdlib>
#include <iostream>

// pcl includes
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// pcl ros
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp> 

int main(int argc, char** argv)
{

    // initialize ROS boilerplate for publishers
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    typedef sensor_msgs::Image Image;
    //ros::Publisher kinect_image_pub = n.advertise<sensor_msgs::Image>("kinect_images", 1);
    ros::Publisher kinect_rgb_pub = n.advertise<Image>("kinect_rgb", 1);
    ros::Publisher kinect_depth_pub = n.advertise<Image>("kinect_depth", 1);

   
    // should be a similar procedure for publishing depth, except need to change the encoding, etc. 
    // ros:Publisher kinect_depth_pub = n.advertise<sensor_msgs::Image>("kinect_depth", 1);

    ros::Rate loop_rate(1); // hz

    //////////////////////////////////
    // add libfreenect2 boilerplate //
    //////////////////////////////////

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;

    std::string serial = "";        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(w, h));
        
        // cloud = updateCloud(undistorted, registered, cloud);

        // pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);
    bool enable_rgb = true;
    bool enable_depth = true;
    bool protonect_shutdown = false;

    size_t framecount = 0;

    int deviceId = -1;
    size_t framemax = 15;
    if(freenect2.enumerateDevices() == 0)
    {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }
    if (serial == "")
    {
        serial = freenect2.getDefaultDeviceSerialNumber();
    }

    pipeline = new libfreenect2::CudaPacketPipeline(0);
    dev = freenect2.openDevice(serial, pipeline);
    int types = 0;
    if (enable_rgb){
        types |= libfreenect2::Frame::Color;
    }
    if (enable_depth)
        types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
    libfreenect2::SyncMultiFrameListener listener(types);
    libfreenect2::FrameMap frames;
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);

    if (enable_rgb && enable_depth)
    {
        if (!dev->start())
            return -1;   
    }
    else
    {
        if(!dev->startStreams(enable_rgb, enable_depth))
            return -1;        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(w, h));
        
        // cloud = updateCloud(undistorted, registered, cloud);

        // pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);

    }
    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

    ROS_INFO("IR_PARAMS_fx: %f", dev->getIrCameraParams().fx);
    ROS_INFO("IR_PARAMS_fy: %f", dev->getIrCameraParams().fy);
    ROS_INFO("IR_PARAMS_cx: %f", dev->getIrCameraParams().cx);
    ROS_INFO("IR_PARAMS_cy: %f", dev->getIrCameraParams().cy);
    ROS_INFO("IR_PARAMS_k1: %f", dev->getIrCameraParams().k1);
    ROS_INFO("IR_PARAMS_k2: %f", dev->getIrCameraParams().k2);
    ROS_INFO("IR_PARAMS_k3: %f", dev->getIrCameraParams().k3);
    ROS_INFO("IR_PARAMS_p1: %f", dev->getIrCameraParams().p1);
    ROS_INFO("IR_PARAMS_p2: %f", dev->getIrCameraParams().p2);

    ROS_INFO("COLOR_PARAMS_fx: %f", dev->getColorCameraParams().fx);
    ROS_INFO("COLOR_PARAMS_fy: %f", dev->getColorCameraParams().fy);
    ROS_INFO("COLOR_PARAMS_cx: %f", dev->getColorCameraParams().cx);
    ROS_INFO("COLOR_PARAMS_cy: %f", dev->getColorCameraParams().cy);
    

    // Camera Info from Kinect Factory Settings
    libfreenect2::Freenect2Device::IrCameraParams IR_params;

    IR_params = {367.629913, 367.629913, 261.900604, 208.773300, 0.087892, -0.271278, 0.096343, 0.000000, 0.000000};

    libfreenect2::Freenect2Device::ColorCameraParams color_params;

    color_params = {1081.372070, 1081.372070, 959.500000, 539.500000};

    while (ros::ok() && !protonect_shutdown && (framemax == (size_t)-1 || framecount < framemax ))
    {
        sensor_msgs::Image msgRgb; 
        sensor_msgs::Image msgDepth;

        // retrieve images from kinect
        if (!listener.waitForNewFrame(frames, 10*1000)) // 10 seconds
        {
            std::cout << "timeout!" << std::endl;
            return -1;
        }
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        registration->apply(rgb, depth, &undistorted, &registered);
        
        cv::Mat matDepth, matColor;
        cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(matDepth);
        cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(matColor);


        // look at createHeader in https://github.com/code-iai/iai_kinect2/blob/master/kinect2_bridge/src/kinect2_bridge.cpp#L1374
        msgRgb.header.seq = 0;
        msgRgb.header.stamp = ros::Time::now(); // rgb->timestamp;
        msgRgb.header.frame_id = "table_frame";

        msgRgb.height = int(rgb->height);
        msgRgb.width = int(rgb->width);
        
        // this one definitely is wrong but fix later
        msgRgb.encoding = "8UC4";

        msgRgb.is_bigendian = false;
        msgRgb.step = matColor.elemSize() * matColor.cols;
        msgRgb.data.resize(matColor.cols * matColor.elemSize() * matColor.rows);
        memcpy(msgRgb.data.data(), matColor.data, matColor.cols * matColor.elemSize() * matColor.rows);

        msgDepth.header.seq = 0;
        msgDepth.header.stamp = msgRgb.header.stamp; // rgb->timestamp;
        msgDepth.header.frame_id = "table_frame";

        msgDepth.height = int(depth->height);
        msgDepth.width = int(depth->width);
        
        // this one definitely is wrong but fix later
        msgDepth.encoding = "32FC1";

        msgDepth.is_bigendian = false;
        msgDepth.step = matDepth.elemSize() * matDepth.cols;
        msgDepth.data.resize(matDepth.cols * matDepth.elemSize() * matDepth.rows);
        memcpy(msgDepth.data.data(), matDepth.data, matDepth.cols * matDepth.elemSize() * matDepth.rows);

        
        listener.release(frames);

        // publish to the topic 
        //kinect_image_pub.publish(img);
        kinect_rgb_pub.publish(msgRgb);
        kinect_depth_pub.publish(msgDepth);

        // kinect_depth_pub.publish(msgDepth);

    }

    dev->stop();
    dev->close();

}

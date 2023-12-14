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


int main(int argc, char** argv)
{

    // initialize ROS boilerplate for publishers
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Publisher kinect_image_pub = n.advertise<std_msgs::Image>("kinect_images", 1);
    
    // should be a similar procedure for publishing depth, except need to change the encoding, etc. 
    // ros:Publisher kinect_depth_pub = n.advertise<std_msgs::Image>("kinect_depth", 1);

    ros::Rate loop_rate(10);

    //////////////////////////////////
    // add libfreenect2 boilerplate //
    //////////////////////////////////

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;

    std::string serial = "";
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
            return -1;

    }
    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);


    while (ros::ok() && !protonect_shutdown && (framemax == (size_t)-1 || framecount < framemax ))
    {
        sensor_msgs::Image img; 

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
        
        img.header.seq = rgb->sequence;
        img.header.stamp = rgb->timestamp;
        img.header.frame_id = "frame_1";

        img.height = int(rgb->height);
        img.width = int(rgb->width);
        
        // this one definitely is wrong but fix later
        img.encoding = rgb->format;

        img.is_bigendian = false;
        img.step = 3 * rgb->width;

        listener.release(frames);
        //framecount++;

        // get header information... timestamp, sequence, frame_id

        // get height, width, encoding, is_bigendian, step, and data from libfreenect2 frame
        // a libfreenect2::Frame is almost a 1 to 1 for a sensor_msgs::image




        // publish to the topic 
        kinect_image_pub.publish(img);


    }

    dev->stop();
    dev->close();

}

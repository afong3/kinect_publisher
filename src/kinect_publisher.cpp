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

int main(int argc, char** argv)
{

    // initialize ROS boilerplate for publishers
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
    //ros::Publisher kinect_image_pub = n.advertise<sensor_msgs::Image>("kinect_images", 1);
    ros::Publisher kinect_cloud_pub = n.advertise<PointCloud>("kinect_clouds", 1);
   
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

    // libfreenect2::Freenect2Device::IrCameraParams IR_params = dev->getIrCameraParams() ;
    // libfreenect2::Freenect2Device::ColorCameraParams color_params = dev->getColorCameraParams();

    libfreenect2::Freenect2Device::IrCameraParams IR_params;

    IR_params = {367.629913, 367.629913, 261.900604, 208.773300, 0.087892, -0.271278, 0.096343, 0.000000, 0.000000};

    libfreenect2::Freenect2Device::ColorCameraParams color_params;

    color_params = {1081.372070, 1081.372070, 959.500000, 539.500000, 863.000000, 52.000000, 0.000663, -0.000024, -0.000046, 0.000505, -0.000305, -0.000219, 0.000740, 0.634050, -0.000641, 0.150087, 0.000026, 0.000847, 0.000550, 0.000078, 0.000091, 0.000805, -0.000293, 0.000646, 0.634342, -0.040878};


    // libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    libfreenect2::Registration* registration = new libfreenect2::Registration(IR_params, color_params);
    
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

    ROS_INFO("IR_PARAMS_fx: %f", IR_params.fx);
    ROS_INFO("IR_PARAMS_fy: %f", IR_params.fy);
    ROS_INFO("IR_PARAMS_cx: %f", IR_params.cx);
    ROS_INFO("IR_PARAMS_cy: %f", IR_params.cy);
    ROS_INFO("IR_PARAMS_k1: %f", IR_params.k1);
    ROS_INFO("IR_PARAMS_k2: %f", IR_params.k2);
    ROS_INFO("IR_PARAMS_k3: %f", IR_params.k3);
    ROS_INFO("IR_PARAMS_p1: %f", IR_params.p1);
    ROS_INFO("IR_PARAMS_p2: %f", IR_params.p2);

    ROS_INFO("COLOR_PARAMS_fx: %f", color_params.fx);
    ROS_INFO("COLOR_PARAMS_fy: %f", color_params.fy);
    ROS_INFO("COLOR_PARAMS_cx: %f", color_params.cx);
    ROS_INFO("COLOR_PARAMS_cy: %f", color_params.cy);
    ROS_INFO("COLOR_PARAMS_shift_d: %f", color_params.shift_d);
    ROS_INFO("COLOR_PARAMS_shift_m: %f", color_params.shift_m);
    ROS_INFO("COLOR_PARAMS_mx_x3y0: %f", color_params.mx_x3y0);
    ROS_INFO("COLOR_PARAMS_mx_x0y3: %f", color_params.mx_x0y3);
    ROS_INFO("COLOR_PARAMS_mx_x2y1: %f", color_params.mx_x2y1);
    ROS_INFO("COLOR_PARAMS_mx_x1y2: %f", color_params.mx_x1y2);
    ROS_INFO("COLOR_PARAMS_mx_x2y0: %f", color_params.mx_x2y0);
    ROS_INFO("COLOR_PARAMS_mx_x0y2: %f", color_params.mx_x0y2);
    ROS_INFO("COLOR_PARAMS_mx_x1y1: %f", color_params.mx_x1y1);
    ROS_INFO("COLOR_PARAMS_mx_x1y0: %f", color_params.mx_x1y0);
    ROS_INFO("COLOR_PARAMS_mx_x0y1: %f", color_params.mx_x0y1);
    ROS_INFO("COLOR_PARAMS_mx_x0y0: %f", color_params.mx_x0y0);
    ROS_INFO("COLOR_PARAMS_my_x3y0: %f", color_params.my_x3y0);
    ROS_INFO("COLOR_PARAMS_my_x0y3: %f", color_params.my_x0y3);
    ROS_INFO("COLOR_PARAMS_my_x2y1: %f", color_params.my_x2y1);
    ROS_INFO("COLOR_PARAMS_my_x1y2: %f", color_params.my_x1y2);
    ROS_INFO("COLOR_PARAMS_my_x2y0: %f", color_params.my_x2y0);
    ROS_INFO("COLOR_PARAMS_my_x0y2: %f", color_params.my_x0y2);
    ROS_INFO("COLOR_PARAMS_my_x1y1: %f", color_params.my_x1y1);
    ROS_INFO("COLOR_PARAMS_my_x1y0: %f", color_params.my_x1y0);
    ROS_INFO("COLOR_PARAMS_my_x0y1: %f", color_params.my_x0y1);
    ROS_INFO("COLOR_PARAMS_my_x0y0: %f", color_params.my_x0y0);


    

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
        img.header.stamp = ros::Time::now(); // rgb->timestamp;
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

        // converting to PCL point cloud, code from https://github.com/OpenKinect/libfreenect2/issues/722
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        PointCloud::Ptr cloud (new PointCloud);
        cloud->header.frame_id = "world";
        cloud->reserve(512 * 424);
        pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
        for(int r = 0; r < 424; ++r)
        {
            for(int c = 0; c < 512; ++c)
            {
                pcl::PointXYZRGB pt;
                registration->getPointXYZRGB(&undistorted, &registered, r, c, pt.x, pt.y, pt.z, pt.rgb);
                cloud->push_back(pt);
            }
        }
        pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);

        // publish to the topic 
        //kinect_image_pub.publish(img);
        kinect_cloud_pub.publish(*cloud);

    }

    dev->stop();
    dev->close();

}

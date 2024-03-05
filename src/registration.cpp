// ROS includes
#include <image_transport/image_transport.h>
//#include <opencv2/highgui/highgui.hpp>
// #include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sstream> // for converting the command line parameter to integer

// libfreenect2 includes
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/frame_listener.hpp>
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

// Message filters
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h> 

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp> 

//cv_bridge
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Topics
static const std::string COLOR_TOPIC = "/kinect_rgb";
static const std::string DEPTH_TOPIC = "/kinect_depth";
//static const std::string COLOR_TOPIC = "";
static const std::string PUBLISH_TOPIC = "/kinect_registered";

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;


// ROS Publisher 
ros::Publisher pub;

void callback(const sensor_msgs::ImageConstPtr& color_msg, const sensor_msgs::ImageConstPtr& depth_msg, libfreenect2::Registration* registration)
{
        ROS_INFO("hmmmm");

        libfreenect2::Frame undistorted = libfreenect2::Frame(512, 424, 4);
        libfreenect2::Frame registered = libfreenect2::Frame(512, 424, 4);

        cv_bridge::CvImagePtr cv_ptr_color;

        try{
            cv_ptr_color = cv_bridge::toCvCopy(color_msg, "8UC4");
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv_bridge::CvImagePtr cv_ptr_depth;

        try{
            cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, "32FC1");
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Create libfreenect2::Frame's to use the Registration method
        libfreenect2::Frame frame_color(color_msg->width, color_msg->height, 1);
        libfreenect2::Frame frame_depth(depth_msg->width, depth_msg->height, 4);

        frame_color.data = cv_ptr_color->image.data;
        frame_depth.data = cv_ptr_depth->image.data;

        registration->apply(&frame_color, &frame_depth, &undistorted, &registered);

        // pcl_conversions::moveFromPCL(cloud_filtered, output);
        PointCloud::Ptr cloud (new PointCloud);
        cloud->header.frame_id = "table_frame";
        cloud->reserve(512 * 424);
        pcl_conversions::toPCL(color_msg->header.stamp, cloud->header.stamp);
        for(int r = 0; r < 424; ++r)
        {
            for(int c = 0; c < 512; ++c)
            {
                pcl::PointXYZRGB pt;
                registration->getPointXYZRGB(&undistorted, &registered, r, c, pt.x, pt.y, pt.z, pt.rgb);
                cloud->push_back(pt);
            }
        }
        // Publish the data
        pub.publish (cloud);
}

int main(int argc, char **argv)
{
  // Initialize the ROS Node "roscpp_pcl_example"
    ros::init (argc, argv, "registration");
    ros::NodeHandle nh;

    // Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
    pub = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);

    libfreenect2::Freenect2Device::IrCameraParams IR_params;
    libfreenect2::Freenect2Device::ColorCameraParams color_params;
    IR_params = {367.629913, 367.629913, 261.900604, 208.773300, 0.087892, -0.271278, 0.096343, 0.000000, 0.000000};
    color_params = {1081.372070, 1081.372070, 959.500000, 539.500000};
    libfreenect2::Registration* registration = new libfreenect2::Registration(IR_params, color_params);
    
    // Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a callback function to cloud_cb
    message_filters::Subscriber<sensor_msgs::Image> color_sub(nh, COLOR_TOPIC, 5);
    
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, DEPTH_TOPIC, 5);
    
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(color_sub, depth_sub, 5); 
    
    // sync.registerCallback(boost::bind(&Node::callback, node, _1, _2));
    sync.registerCallback(boost::bind(&callback, _1, _2, registration));

    ROS_INFO("7");
   


    // Spin
    ros::spin();

    // Success
    return 0;
}
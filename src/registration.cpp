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

// // opencv
// #include <opencv2/core/core.hpp>
// #include <opencv2/imgcodecs.hpp> 

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

// void callback(const sensor_msgs::ImageConstPtr& color_msg, const sensor_msgs::ImageConstPtr& depth_msg, libfreenect2::Registration* registration)
void callback(const sensor_msgs::ImageConstPtr& color_msg, const sensor_msgs::ImageConstPtr& depth_msg, const libfreenect2::Registration* registration)
{
        libfreenect2::Frame undistorted = libfreenect2::Frame(512, 424, 4);
        libfreenect2::Frame registered = libfreenect2::Frame(512, 424, 4);

        cv_bridge::CvImageConstPtr cv_ptr_color;
        cv::Mat mat_color;

        try{
            cv_ptr_color = cv_bridge::toCvShare(color_msg, "bgra8");

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

        libfreenect2::Frame *frame_color = new libfreenect2::Frame(1920, 1080, 4, cv_ptr_color->image.data);
        libfreenect2::Frame *frame_depth = new libfreenect2::Frame(512, 424, 4, cv_ptr_depth->image.data);
        // invalid format
        frame_color->format = libfreenect2::Frame::Format::BGRX;
        frame_depth->format = libfreenect2::Frame::Format::Float;

        registration->apply(frame_color, frame_depth, &undistorted, &registered);

        PointCloud::Ptr cloud (new PointCloud);
        cloud->header.frame_id = "table_frame";
        cloud->reserve(512 * 424);
        pcl_conversions::toPCL(color_msg->header.stamp, cloud->header.stamp);
        int k = 0;
        for(int r = 0; r < 424; ++r)
        {
            k++;
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

    IR_params = {367.629913, 367.629913, 261.900604, 208.773300, 0.087892, -0.271278, 0.096343, 0.000000, 0.000000};

    libfreenect2::Freenect2Device::ColorCameraParams color_params;

    color_params = {1081.372070, 1081.372070, 959.500000, 539.500000, 863.000000, 52.000000, 0.000663, -0.000024, -0.000046, 0.000505, -0.000305, -0.000219, 0.000740, 0.634050, -0.000641, 0.150087, 0.000026, 0.000847, 0.000550, 0.000078, 0.000091, 0.000805, -0.000293, 0.000646, 0.634342, -0.040878};

    libfreenect2::Registration* registration = new libfreenect2::Registration(IR_params, color_params);
    
    // Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a callback function to cloud_cb
    message_filters::Subscriber<sensor_msgs::Image> color_sub(nh, COLOR_TOPIC, 5);
    
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, DEPTH_TOPIC, 5);
    
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(color_sub, depth_sub, 5); 
    
    // sync.registerCallback(boost::bind(&Node::callback, node, _1, _2));
    sync.registerCallback(boost::bind(&callback, _1, _2, registration));

    // Spin
    ros::spin();

    // Success
    return 0;
}
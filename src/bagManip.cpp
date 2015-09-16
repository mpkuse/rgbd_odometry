/// @file Subscribes to RGB-D from the bag files and retransmits them with manipulation. Manipulations will include frame skipping, adding noise, changing illumination


#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

ros::Subscriber sub_rgb; int rgb_count=0;
ros::Subscriber sub_depth; int depth_count=0;

ros::Publisher pub_rgb;
ros::Publisher pub_depth;

int SKIP = 5;

void imageCallBack(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
//        cv::imshow("manipulator view", cv_bridge::toCvShare(msg, "bgr8")->image);
//        cv::waitKey(30);

        if( rgb_count%SKIP == 0 )
            pub_rgb.publish( msg );

        rgb_count++;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void depthCallBack(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        //cv::imshow("view", cv_bridge::toCvShare(msg)->image);
        //cv::waitKey(30);

        if( depth_count%SKIP == 0 )
            pub_depth.publish( msg );

        depth_count++;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


int main( int argc, char ** argv )
{
    ros::init( argc, argv, "stream_manip" );
    ros::NodeHandle nh;
    ROS_INFO( "SKIP=%d", SKIP );

     sub_rgb   = nh.subscribe("/camera/rgb/image_color", 10, &imageCallBack );
     sub_depth = nh.subscribe("/camera/depth/image", 10, &depthCallBack );

     pub_rgb = nh.advertise<sensor_msgs::Image>( "/modStream/rgb/image_color", 10 );
     pub_depth = nh.advertise<sensor_msgs::Image>( "/modStream/depth/image", 10 );

    ros::spin();
}

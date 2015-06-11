#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


#include <rgbd_odometry/RGBDFrame.h>

#include <RGBDOdometry.h>


//void custom_image_rcvd( const rgbd_odometry::RGBDFrameConstPtr& msg )
//{
//    //ROS_INFO_STREAM( "msg::: Name:"<< msg->name << " Age:"<< msg->age );


//    cv::Mat frame =  cv_bridge::toCvShare(  msg->frame, msg, "bgr8" )->image ;
//    cv::Mat dframe =  cv_bridge::toCvShare(  msg->dframe, msg, "mono16" )->image ;
//    ROS_INFO_STREAM( dframe.at<uint16_t>(200,200) );
//    cv::imshow("viewz", frame );
//    cv::imshow("viewz_depth", dframe );
//    cv::waitKey(30);
//}


int main( int argc, char ** argv )
{
    ros::init(argc, argv, "rgbd_subsc");

    RGBDOdometry odo;
    odo.eventLoop();


}

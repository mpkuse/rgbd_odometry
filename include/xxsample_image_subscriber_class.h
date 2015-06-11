/**
  SolvePnP.h

  Author : Manohar Kuse
  Date   : 3rd May, 2015

  Defines the class to handle streaming images of the chess board and compute 3d-2d PnP
  */


#ifndef ___RGB_ODOMETRY_H___
#define ___RGB_ODOMETRY_H___

#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


#include <rgbd_odometry/RGBDFrame.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>




/// Defines the class to handle streaming images and compute 3d-2d PnP of chess board
class SolvePnP
{
public:
    SolvePnP();
    void eventLoop();

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;

    void imageArrivedCallBack( rgbd_odometry::RGBDFrameConstPtr msg );

    cv::Mat rcvd_frame, rcvd_dframe; ///< Received RGBD data
    bool isFrameAvailable;


};





#endif //___RGB_ODOMETRY_H___

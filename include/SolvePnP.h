/**
  SolvePnP.h

  Author : Manohar Kuse
  Date   : 3rd May, 2015

  Defines the class to handle streaming images of the chess board and compute 3d-2d PnP
  */


#ifndef ___RGB_ODOMETRY_H___
#define ___RGB_ODOMETRY_H___

#include <ros/ros.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>


#include <rgbd_odometry/RGBDFrame.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>






/// Defines the class to handle streaming images and compute 3d-2d PnP of chess board
class SolvePnP
{
public:
    SolvePnP();
    void eventLoop();

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub_markers;   ///< markers of 3d pts(array)
    ros::Publisher pub_pose;      ///< intermediate poses during iterations
    ros::Publisher pub_pose_1;
    ros::Publisher pub_path;      ///< Array of poses (path)
    std::vector<geometry_msgs::Pose> prevPoses;
    ros::Publisher pub_im_reproj; ///< Image reprojection

    void imageArrivedCallBack( rgbd_odometry::RGBDFrameConstPtr msg );

    cv::Mat rcvd_frame, rcvd_dframe; ///< Received RGBD data
    bool isFrameAvailable;


    bool getChessBoardPts( Eigen::MatrixXd& objPts, Eigen::MatrixXd& imPts );

    void PnP( Eigen::MatrixXd& objPts, Eigen::MatrixXd& imPts, Eigen::Matrix3d& R, Eigen::Vector3d& T );
    void opencvPnP( Eigen::MatrixXd& objPts, Eigen::MatrixXd& imPts, Eigen::Matrix3d& R, Eigen::Vector3d& T );



    // helpers of PnP
    void computeJacobian( int index, Eigen::MatrixXd& objPts, Eigen::MatrixXd& imPts, Eigen::Matrix3d& R, Eigen::Vector3d& T, Eigen::MatrixXd& J_i );
    void computeResidue(  int index, Eigen::MatrixXd& objPts, Eigen::MatrixXd& imPts, Eigen::Matrix3d& R, Eigen::Vector3d& T, Eigen::Vector2d &r_i );
    void to_se_3(Eigen::Vector3d& w, Eigen::Matrix3d& wx);
    void exponentialMap(Eigen::VectorXd& psi, Eigen::Matrix3d& outR, Eigen::Vector3d& outT);

    std::vector<Eigen::Matrix3d> stored_R;
    std::vector<Eigen::Vector3d> stored_T;

    void getReprojectedImage( Eigen::MatrixXd& objectPts, Eigen::Matrix3d R, Eigen::Vector3d T, cv::Mat& imReproj );





    void setCameraMatrix(char* calibFile);
    // Camera params
    bool cameraIntrinsicsReady;
    cv::Mat cameraMatrix, distCoeffs;
    Eigen::Matrix3d K; //same as cameraMatrix
    double fx, fy, cx, cy;



    void publish3dPoints( Eigen::MatrixXd objectPts );
    void publishPoses(bool publishFinalPose, bool publishIntermediatePoses);


};





#endif //___RGB_ODOMETRY_H___

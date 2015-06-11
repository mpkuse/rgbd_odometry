/**
  RGBDOdometry.h

  Author : Manohar Kuse
  Date   : 14th Apr, 2015

  Defines the class to handle streaming images and compute the dense odometry

  */


#ifndef ___RGB_ODOMETRY_H___
#define ___RGB_ODOMETRY_H___

#include <vector>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


#include <rgbd_odometry/RGBDFrame.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>


typedef Eigen::Transform<double, 3, Eigen::Affine> TransformRep;
//typedef Eigen::Matrix4d TransformRep;
//typedef Eigen::Transform<double, 3, Eigen::Projective> TransformRep;

/// Defines the class to handle streaming images and compute the dense odometry
class RGBDOdometry {

public:
    RGBDOdometry();

    void eventLoop();

private:
    ros::NodeHandle nh;

    ros::Subscriber sub; ///< subscriber of RGBDFrame stream

    ros::Publisher odom_pub; ///< odometry data publisher
    ros::Publisher path_pub; ///< path data publisher
    ros::Publisher pose_pub; ///< path data publisher

    void imageArrivedCallBack( rgbd_odometry::RGBDFrameConstPtr msg );


    void setCameraMatrix( char * calibFile );
    // Camera params
    bool cameraIntrinsicsReady;
    cv::Mat cameraMatrix, distCoeffs;
    double fx, fy, cx, cy;




    cv::Mat rcvd_frame, rcvd_dframe; ///< Received RGBD data
    bool isFrameAvailable;

    //display helpers
    void showFrames();
    void sOverlay( cv::Mat im, Eigen::MatrixXi mask );


    ///////////////////// Storage of `Reference` and `Now` frames ////////////////////
    /// Reference frames in color, gray, depthMap
    cv::Mat im_r_color, im_r, dim_r;
    bool isRefFrameAvailable;


    /// Now frames in color, gray, depthMap
    cv::Mat im_color, im, dim;
    bool isNowFrameAvailable;


    /// Reference frames at each pyramidal levels
    std::vector<cv::Mat> _im_r_color, _im_r, _dim_r;
    bool isPyramidalRefFrameAvailable;

    /// Now frames at each pyramidal levels
    std::vector<cv::Mat> _im_color, _im, _dim;
    bool isPyramidalNowFrameAvailable;


    /// Storage for Jacobians
    std::vector<Eigen::MatrixXd> _J, _A; //A := J'*J
    bool isJacobiansAvailable;

    /// Storage of mask (of pixels with higher im_grad
    std::vector<Eigen::MatrixXi> _roimask;



    /// Set functions. Note. These functions do a deepcopy into the above variables
    void setRefFrame( cv::Mat rgb, cv::Mat depth );
    void setNowFrame( cv::Mat rgb, cv::Mat depth );

    /////////////////////////////////////////////////////////////////////////////////


    // Constants
    int const_gradientThreshold;
    int const_maxJacobianSize;
    int const_minimumRequiredPts;

    void computeJacobianAllLevels(); //this function is supposed to be called when ref frame gets updated
    void computeJacobian( int level, Eigen::MatrixXd &J, Eigen::MatrixXi &semiDenseMarkings );


    void gaussNewtonIterations(int level, TransformRep& T);
    void computeEpsilon( int level, TransformRep T, Eigen::VectorXd &epsilon, Eigen::MatrixXi &newroimask );

    void exponentialMap(Eigen::VectorXd& psi, Eigen::Matrix4d& outTr);
    void to_se_3(Eigen::Vector3d& w, Eigen::Matrix3d& wx);



};




















#endif // ___RGB_ODOMETRY_H___

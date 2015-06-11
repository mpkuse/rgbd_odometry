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
#include <rgbd_odometry/RGBDFramePyd.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>



//#define GRAD_NORM( A, B ) fabs(A) + fabs(B)
#define GRAD_NORM( A, B ) fabs(A)

typedef std::vector<Eigen::RowVectorXf> JacobianList;



/// Defines the class to handle streaming images and compute 3d-2d PnP of chess board
class SolveDVO
{
public:
    SolveDVO();
    void loop();

    void setCameraMatrix(const char* calibFile);


private:

    //
    // Publishers & Subscribers
    ros::NodeHandle nh;
    ros::Subscriber sub;

    ros::Publisher pub_pc;
    ros::Publisher pub_final_pose;


    //
    // Camera params
    bool isCameraIntrinsicsAvailable;
    cv::Mat cameraMatrix, distCoeffs;
    Eigen::Matrix3f K, K_inv; //same as cameraMatrix
    float fx, fy, cx, cy;




    void imageArrivedCallBack( rgbd_odometry::RGBDFramePydConstPtr msg );

    //
    // Received Frame
    //cv::Mat rcvd_frame, rcvd_dframe; ///< Received RGBD data
    std::vector<Eigen::MatrixXf> rcvd_framemono, rcvd_depth; ///< Received image pyramid

    bool isFrameAvailable;



    //
    // Now & Ref Frames
    std::vector<Eigen::MatrixXf> im_n, dim_n; ///< Now frames
    std::vector<Eigen::MatrixXf> im_r, dim_r; ///< Reference frames
    void setRefFrame();
    void setNowFrame();
    bool isNowFrameAvailable, isRefFrameAvailable;


    //
    // Functions relating to the Gauss-Newton Iterations
    void computeJacobian(int level, JacobianList &J, Eigen::MatrixXi& roi );
    std::vector<JacobianList> J_r;
    std::vector<Eigen::MatrixXi> ROI_r;
    void gaussNewtonIterations( int level, int maxIterations, Eigen::Matrix3f &cR, Eigen::Vector3f &cT );
    bool computeEpsilon( int level, Eigen::Matrix3f& cR, Eigen::Vector3f& cT, Eigen::MatrixXf &A, Eigen::VectorXf &b );


    // helpers
    void imageGradient( Eigen::MatrixXf &image, Eigen::MatrixXf& gradientX, Eigen::MatrixXf &gradientY );
    void to_se_3(Eigen::Vector3f& w, Eigen::Matrix3f& wx);
    void to_se_3(float w0, float w1, float w2, Eigen::Matrix3f& wx);
    void exponentialMap(Eigen::VectorXf &psi, Eigen::Matrix3f &outR, Eigen::Vector3f &outT);
    void sOverlay( Eigen::MatrixXf eim, Eigen::MatrixXi mask, cv::Mat &outIm, cv::Vec3d color);
    int countSelectedPts(Eigen::MatrixXf Gx, Eigen::MatrixXf Gy);






    //
    //other debuging functions
    void imshowEigenImage(const char *winName, Eigen::MatrixXd eim);
    void imshowEigenImage(const char *winName, Eigen::MatrixXf eim);
    void loadFromFile( const char * xmlFileName );


    //
    //publish
    void publishBowl();
    void publishCurrentPointCloud();
    void publishPose( Eigen::MatrixXf rot, Eigen::VectorXf tran );

    //helpers for publishing
    void matrixToPose( Eigen::Matrix3f rot, Eigen::Vector3f tran, geometry_msgs::Pose &rospose );


    // constants
    float grad_thresh;



};





#endif //___RGB_ODOMETRY_H___

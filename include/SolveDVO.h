/**
  SolvePnP.h

  Author : Manohar Kuse
  Date   : 3rd May, 2015

  Defines the class to handle streaming images of the chess board and compute 3d-2d PnP
  */


#ifndef ___RGB_ODOMETRY_H___
#define ___RGB_ODOMETRY_H___

#include <ros/ros.h>
#include <ros/console.h>

#include <fstream>

#include <sophus/se3.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <igl/repmat.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <rgbd_odometry/RGBDFrame.h>
#include <rgbd_odometry/RGBDFramePyd.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>



//#define GRAD_NORM( A, B ) fabs(A) + fabs(B)
#define GRAD_NORM( A, B ) fabs(A)

#define __SHOW_REPROJECTIONS_EACH_ITERATION__

#define __REPROJECTION_LEVEL 0


typedef std::vector<Eigen::RowVectorXf> JacobianList;
typedef Eigen::MatrixXf ImCordList;  //2xN
typedef Eigen::MatrixXf SpaceCordList; //3xN
//typedef std::vector<float> IntensityList;
typedef Eigen::VectorXf IntensityList;



/// Defines the class to handle streaming images and compute camera pose from RGBD images ie. dense visual odometry (DVO)
class SolveDVO
{
public:
    SolveDVO();
    void loop();

    void setCameraMatrix(const char* calibFile);


private:


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
    std::vector<JacobianList> _J;
    std::vector<ImCordList> _imCord;
    std::vector<SpaceCordList> _spCord;
    std::vector<IntensityList> _intensities;
    std::vector<Eigen::MatrixXi> _roi;
    bool isJacobianComputed;
    void computeJacobian();
    void computeJacobian(int level, JacobianList &J, ImCordList &imC, SpaceCordList &spC, IntensityList &I, Eigen::MatrixXi &refROI );
    void gaussNewtonIterations( int level, int maxIterations, Eigen::Matrix3f &cR, Eigen::Vector3f &cT );
    float computeEpsilon( int level, Eigen::Matrix3f& cR, Eigen::Vector3f& cT, Eigen::MatrixXf &A, Eigen::VectorXf &b );
    void updateEstimates( Eigen::Matrix3f& cR, Eigen::Vector3f& cT, Eigen::Matrix3f& xRot, Eigen::Vector3f& xTrans );
    float getWeightOf( float r );
    bool signalGetNewRefImage;


    // helpers
    void imageGradient( Eigen::MatrixXf &image, Eigen::MatrixXf& gradientX, Eigen::MatrixXf &gradientY );
    void to_se_3(Eigen::Vector3f& w, Eigen::Matrix3f& wx);
    void to_se_3(float w0, float w1, float w2, Eigen::Matrix3f& wx);
    void exponentialMap(Eigen::VectorXf &psi, Eigen::Matrix3f &outR, Eigen::Vector3f &outT);
    void sOverlay( Eigen::MatrixXf eim, Eigen::MatrixXi mask, cv::Mat &outIm, cv::Vec3d color);
    int countSelectedPts(Eigen::MatrixXf& Gx, Eigen::MatrixXf& Gy, Eigen::MatrixXi &roi);
    void printRT( Eigen::Matrix3f &fR, Eigen::Vector3f &fT, const char *msg );
    void visualizeHistogram( Eigen::VectorXf residi );

    // debugging variable
    Eigen::MatrixXi __now_roi_reproj;
    Eigen::VectorXf __residues;




    //
    //other debuging functions
    void imshowEigenImage(const char *winName, Eigen::MatrixXd eim);
    void imshowEigenImage(const char *winName, Eigen::MatrixXf eim);
    void loadFromFile( const char * xmlFileName );


    //
    // Publishers & Subscribers
    ros::NodeHandle nh;
    ros::Subscriber sub;

    ros::Publisher pub_pc; //point-cloud
    ros::Publisher pub_final_pose; // final pose relative to first frame (ie. global frame)
    ros::Publisher pub_pose_wrt_ref; //publish pose with respect to ref frame


    char const * rviz_frame_id;


    //
    //publish
    void publishBowl();
    void publishCurrentPointCloud(int level);
    void publishReferencePointCloud( int level );
    void publishPointCloud( SpaceCordList& spc, IntensityList &grays );
    void publishPoseFinal( Eigen::MatrixXf rot, Eigen::VectorXf tran );
    void publishPoseWrtRef( Eigen::MatrixXf rot, Eigen::VectorXf tran );


    //helpers for publishing
    void matrixToPose( Eigen::Matrix3f rot, Eigen::Vector3f tran, geometry_msgs::Pose &rospose );


    // constants
    float grad_thresh;
    float ratio_of_visible_pts_thresh; //should be between [0, 1]



};





#endif //___RGB_ODOMETRY_H___

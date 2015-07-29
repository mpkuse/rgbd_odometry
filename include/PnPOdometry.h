#ifndef ___PnP_ODOMETRY_H___
#define ___PnP_ODOMETRY_H___

#include <ros/ros.h>

#include <Eigen/Dense>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/core/eigen.hpp>



#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <vector>

#include <rgbd_odometry/RGBDFramePyd.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

class PnPOdometry
{
public:
    PnPOdometry();
    void eventLoop();
    void setCameraMatrix(const char* calibFile);



private:
    //
    // Subscriber & Callback
    ros::NodeHandle nh;
    ros::Subscriber sub_pyd;
    ros::Subscriber sub;

    bool loadFromFile(const char *xmlFileName);
    void imageArrivedCallBack( rgbd_odometry::RGBDFramePydConstPtr msg );



    //
    // Publisher of Pose and related functions
    //      Note: Will publisher under -- "/pnpVO/"
    ros::Publisher pub_rel_pose; ///< relative to ref frame pose
    ros::Publisher pub_global_pose; ///< relative to 1st frame pose (ie. global co-ordinate frame)
    ros::Publisher pub_path;

    void publishRelativePose( cv::Mat iR, cv::Mat iT );
    void publishGlobalPose( cv::Mat iR, cv::Mat iT );

    std::vector<geometry_msgs::PoseStamped> poseAry;
    void publishPath( cv::Mat iR, cv::Mat iT );

    //helper
    void matrixToPose(Eigen::Matrix3f rot, Eigen::Vector3f tran, geometry_msgs::Pose& rospose);




    //
    // Received Frames
    std::vector<cv::Mat> rcvd_frame;
    std::vector<cv::Mat> rcvd_framemono;
    std::vector<cv::Mat> rcvd_dframe;
    bool isFrameAvailable;


    //
    // Camera params
    bool isCameraIntrinsicsAvailable;
    cv::Mat cameraMatrix, distCoeffs;
    float fx, fy, cx, cy;


    //
    // Now Frame
    void setAsNowFrame();
    bool isNowFrameAvailable;
    //std::vector<cv::Mat> now_frame;
    std::vector<cv::Mat> now_framemono;
    //std::vector<cv::Mat> now_dframe;


    //
    // Ref Frame
    void setAsRefFrame();
    bool isRefFrameAvailable;
    std::vector<cv::Mat> ref_frame;
    std::vector<cv::Mat> ref_framemono;
    std::vector<cv::Mat> ref_dframe;


    //
    // Detectors / Extractor / Matcher
    cv::SurfFeatureDetector detector;
    cv::SurfDescriptorExtractor extractor;
    cv::FlannBasedMatcher matcher;

    //
    // Point Features (Ref)
    void extractRefFeature();
    void evalRef3dPoints();
    std::vector<cv::KeyPoint> ref_pt_features;
    std::vector<cv::Point3f> ref_pt_3d;
    cv::Mat ref_pt_descriptors;
    bool isRefFeaturesAvailable;


    //
    // Point Features (Now)
    void extractNowFeature();
    std::vector<cv::KeyPoint> now_pt_features;
    cv::Mat now_pt_descriptors;
    bool isNowFeaturesAvailable;


    //
    // Matches
    void match();
    std::vector< cv::DMatch > all_matches;
    std::vector< cv::DMatch > good_matches;
    bool isMatched;

    // filter matches based on fundamental matrix constraints
    void ransacTest(const std::vector<cv::DMatch> matches,const std::vector<cv::KeyPoint>&keypoints1,const std::vector<cv::KeyPoint>& keypoints2,
                    std::vector<cv::DMatch>& goodMatches,double distance,double confidence);


    //
    // PnP Wraper
    void pnpEstimation( cv::Mat & iR, cv::Mat & iT, std::vector<cv::Point2f> & refPts, std::vector<cv::Point2f> & imagePts, std::vector<cv::Point2f>& reprojectedPts );


    void performerDisplay( std::vector<cv::Point2f> & refPts, std::vector<cv::Point2f> & originalNowImagepts, std::vector<cv::Point2f> &  reprojectedPts );


};





#endif // ___PnP_ODOMETRY_H___

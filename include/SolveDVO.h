/**
  SolvePnP.h

  Author : Manohar Kuse
  Date   : 3rd May, 2015

  Defines the class to handle streaming images of the chess board and compute 3d-2d PnP
  */


#ifndef ___RGB_ODOMETRY_H___
#define ___RGB_ODOMETRY_H___

#define EIGEN_DONT_PARALLELIZE

#include <ros/ros.h>
#include <ros/console.h>

#include <fstream>
#include <iostream>
#include <string>

#include <sophus/se3.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <igl/repmat.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <rgbd_odometry/RGBDFramePyd.h>



#include <FColorMap.h>
#include <MentisVisualHandle.h>


#define GRAD_NORM( A, B ) (fabs(A) + fabs(B))
//#define GRAD_NORM( A, B ) fabs(A)

//#define __SHOW_REPROJECTIONS_EACH_ITERATION__
#define __ENABLE_DISPLAY__   120 //display in loop()



#define __REPROJECTION_LEVEL 0






typedef std::vector<Eigen::RowVectorXf> JacobianList;
typedef Eigen::MatrixXf ImCordList;  //2xN
typedef Eigen::MatrixXf SpaceCordList; //3xN
//typedef std::vector<float> IntensityList;
typedef Eigen::VectorXf IntensityList;
typedef Eigen::MatrixXf JacobianLongMatrix;


// inverse formulation of dist-transform
typedef Eigen::Matrix<float, 1, 6> RowVector6f;


/// Defines the class to handle streaming images and compute camera pose from RGBD images ie. dense visual odometry (DVO)
class SolveDVO
{
    friend class MentisVisualHandle;

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
    //void computeJacobian();
    //void computeJacobian(int level, JacobianList &J, ImCordList &imC, SpaceCordList &spC, IntensityList &I, Eigen::MatrixXi &refROI );
    void gaussNewtonIterations( int level, int maxIterations, Eigen::Matrix3f &cR, Eigen::Vector3f &cT,
                                Eigen::VectorXf& energyAtEachIteration, Eigen::VectorXf& finalEpsilons,
                                Eigen::MatrixXf& finalReprojections, int& bestEnergyIndex, float & finalVisibleRatio );
    float computeEpsilon( int level, Eigen::Matrix3f& cR, Eigen::Vector3f& cT, Eigen::MatrixXf &A, Eigen::VectorXf &b );
    void updateEstimates( Eigen::Matrix3f& cR, Eigen::Vector3f& cT, Eigen::Matrix3f& xRot, Eigen::Vector3f& xTrans );

    float getWeightOf( float r );
    int selectedPts(int level, Eigen::MatrixXi &roi);

    bool signalGetNewRefImage;
    char signalGetNewRefImageMsg[500];


    // helpers
    void imageGradient( Eigen::MatrixXf &image, Eigen::MatrixXf& gradientX, Eigen::MatrixXf &gradientY );
    void to_se_3(Eigen::Vector3f& w, Eigen::Matrix3f& wx);
    void to_se_3(float w0, float w1, float w2, Eigen::Matrix3f& wx);
    void exponentialMap(Eigen::VectorXf &psi, Eigen::Matrix3f &outR, Eigen::Vector3f &outT);
    void sOverlay( Eigen::MatrixXf eim, Eigen::MatrixXi mask, cv::Mat &outIm, cv::Vec3b color);
    void printRT( Eigen::Matrix3f &fR, Eigen::Vector3f &fT, const char *msg );
    float processResidueHistogram( Eigen::VectorXf &residi, bool quite );
    void visualizeResidueHeatMap( Eigen::MatrixXf& eim, Eigen::MatrixXf& residueAt );
    void visualizeReprojectedDepth( Eigen::MatrixXf& eim, Eigen::MatrixXf& reprojDepth );


    // debugging variable
    Eigen::MatrixXi __now_roi_reproj; //this is a `binary` mask of reprojected points
    Eigen::VectorXf __residues;       //list of residue values, -1 ==> this pixel is not visible in reprojected frame
    Eigen::MatrixXf __now_roi_reproj_values; //this is a mask with float values (= residues at that point)
    Eigen::MatrixXf __reprojected_depth; //mask of float values denoting depth values (in mm) in now frames
    Eigen::MatrixXf __weights;


    // distance transform related
    bool isRefDistTransfrmAvailable;
    std::vector<Eigen::MatrixXf> ref_distance_transform;
    std::vector<Eigen::MatrixXi> ref_edge_map;
    std::vector<Eigen::MatrixXf> ref_DT_gradientX;
    std::vector<Eigen::MatrixXf> ref_DT_gradientY;
    void computeDistTransfrmOfRef();

    bool isNowDistTransfrmAvailable;
    std::vector<Eigen::MatrixXf> now_distance_transform;
    std::vector<Eigen::MatrixXi> now_edge_map;
    std::vector<Eigen::MatrixXf> now_DT_gradientX;
    std::vector<Eigen::MatrixXf> now_DT_gradientY;
    void computeDistTransfrmOfNow();

    void visualizeDistanceResidueHeatMap(Eigen::MatrixXf& eim, Eigen::MatrixXi& reprojectionMask, Eigen::MatrixXf& nowDistMap );
    void visualizeEnergyProgress( Eigen::VectorXf energyAtEachIteration, int bestEnergyIndex, int XSPACING=1 );





    //
    //other debuging functions
    void imshowEigenImage(const char *winName, Eigen::MatrixXd& eim);
    void imshowEigenImage(const char *winName, Eigen::MatrixXf& eim);
    bool loadFromFile( const char * xmlFileName );
    void printFrameIndex2Scratch( cv::Mat& scratch, long nowIndx, long lastRef, double time4Jacobian, double time4Iteration, bool cleanScratch  );
    std::string cvMatType2str(int type);



    //
    // Forward formulation (distance energy function) related function (8th July, 2015)
    // selectedPts //< already declared above
    std::vector<SpaceCordList> _ref_edge_3d; ///< Stores the edge points of the reference frame in 3d ie. `E_i`
    std::vector<ImCordList> _ref_edge_2d; ///< Stores edge image co-ordinates (2d) ie. `e_i`
    std::vector<Eigen::MatrixXi> _ref_roi_mask; ///< Reference selected edge points


    // void gaussNewton <-- defined above
    void enlistRefEdgePts( int level, Eigen::MatrixXi &refEdgePtsMask, SpaceCordList& _3d, ImCordList& _2d );
    void preProcessRefFrame();
    void computeJacobianOfNowFrame( int level, Eigen::Matrix3f& cR, Eigen::Vector3f& cT, JacobianLongMatrix& Jcbian, Eigen::MatrixXf &reprojections );
    float getReprojectedEpsilons( int level, Eigen::MatrixXf &reprojections, Eigen::VectorXf &epsilon, Eigen::VectorXf &W );

    void cordList_2_mask( Eigen::MatrixXf& list, Eigen::MatrixXi &mask); //make sure mask is preallocated



    //
    // Publishers & Subscribers
    ros::NodeHandle nh;
    ros::Subscriber sub;

    MentisVisualHandle mviz;


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
    float laplacianThreshExitCond; //set this to typically 15-20
    float psiNormTerminationThreshold; //terminate iteration if norm of psi falls below this value



};





#endif //___RGB_ODOMETRY_H___

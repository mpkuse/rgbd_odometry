#ifndef __GOP_H___
#define __GOP_H___

///////////////////////////////////////////////////////////////////////////
//                        GOP (Group of Pictures)                        //
///////////////////////////////////////////////////////////////////////////
//                                                                       //
//  Author : Manohar Kuse <mpkuse@ust.hk>                                //
//  Created on : 6th Aug, 2015                                           //
//  Purpose : Stores the poses of each frame processed.                  //
//                  As of 6th Aug (morning) there is a bug in the        //
//                    estimates I suspect it to be a crazy memory related//
//                    (lazy evaluation related bug)                      //
//  Updates : Templating the entire GOP/GOPElement class on              //
//                  26th Aug, 2015                                       //
///////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <vector>

#include <Eigen/Dense>

#include <geometry_msgs/Pose.h>






/// Stores the poses of each frame processed and provides an interface for GOP structure
template <typename T>
class GOPElement
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GOPElement();

    void setAsOrdinaryFrame(int frameNum, Eigen::Matrix<T,3,3> wR, Eigen::Matrix<T,3,1> wT );
    void setAsKeyFrame(int frameNum, int reason, Eigen::Matrix<T,3,3> wR, Eigen::Matrix<T,3,1> wT );

    const Eigen::Matrix<T,3,3>& getR();
    const Eigen::Matrix<T,3,1>& getT();
    const geometry_msgs::Pose& getPose();

    bool isKeyFrame();
    int getReason();

    void updateAsKeyFrame(int reason );
private:
    bool keyFrame;
    int frameId;
    int reason_of_change; //this is valid only if isKeyFrame true

    Eigen::Matrix<T,3,3> world_R; //in world-ordinate system (ie. co-ordinate system of 1st frame
    Eigen::Matrix<T,3,1> world_T;

    geometry_msgs::Pose world_pose;
    void matrixToPose(Eigen::Matrix<T,3,3> rot, Eigen::Matrix<T,3,1> tran, geometry_msgs::Pose& rospose);


};



/// Basically contains a vector of `GOPElement`. Need to provide the incremental poses at each frames and
/// denote if it is a keyframe. This class converts all relative poses to global poses
template <typename T>
class GOP
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GOP();

    void pushAsOrdinaryFrame(int frameNum, Eigen::Matrix<T,3,3> cR, Eigen::Matrix<T,3,1> cT );
    void pushAsKeyFrame(int frameNum, int reason, Eigen::Matrix<T,3,3> cR, Eigen::Matrix<T,3,1> cT );

    void updateMostRecentToKeyFrame(int reason);


    int size();

    // Getters
    const geometry_msgs::Pose &getGlobalPoseAt(int i );
    const Eigen::Matrix<T,3,3> &getGlobalRAt(int i );
    const Eigen::Matrix<T,3,1> &getGlobalTAt(int i );

    bool isKeyFrameAt( int i);
    int getReasonAt( int i);

private:
    std::vector< GOPElement<T> > gopVector;

    Eigen::Matrix<T,3,3> lastKeyFr_R;
    Eigen::Matrix<T,3,1> lastKeyFr_T;
};


#endif //__GOP_H___

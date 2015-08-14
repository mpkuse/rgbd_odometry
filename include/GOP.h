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
///////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <vector>

#include <Eigen/Dense>

#include <geometry_msgs/Pose.h>


/// Stores the poses of each frame processed and provides an interface for GOP structure
class GOPElement
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GOPElement();

    void setAsOrdinaryFrame(int frameNum, Eigen::Matrix3f wR, Eigen::Vector3f wT );
    void setAsKeyFrame(int frameNum, int reason, Eigen::Matrix3f wR, Eigen::Vector3f wT );

    const Eigen::Matrix3f& getR();
    const Eigen::Vector3f& getT();
    const geometry_msgs::Pose& getPose();

    bool isKeyFrame();
    int getReason();

    void updateAsKeyFrame(int reason );
private:
    bool keyFrame;
    int frameId;
    int reason_of_change; //this is valid only if isKeyFrame true

    Eigen::Matrix3f world_R; //in world-ordinate system (ie. co-ordinate system of 1st frame
    Eigen::Vector3f world_T;

    geometry_msgs::Pose world_pose;
    void matrixToPose(Eigen::Matrix3f rot, Eigen::Vector3f tran, geometry_msgs::Pose& rospose);


};



/// Basically contains a vector of `GOPElement`. Need to provide the incremental poses at each frames and
/// denote if it is a keyframe. This class converts all relative poses to global poses
class GOP
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GOP();

    void pushAsOrdinaryFrame(int frameNum, Eigen::Matrix3f cR, Eigen::Vector3f cT );
    void pushAsKeyFrame(int frameNum, int reason, Eigen::Matrix3f cR, Eigen::Vector3f cT );

    void updateMostRecentToKeyFrame(int reason);


    int size();

    // Getters
    const geometry_msgs::Pose &getGlobalPoseAt(int i );
    const Eigen::Matrix3f &getGlobalRAt(int i );
    const Eigen::Vector3f &getGlobalTAt(int i );

    bool isKeyFrameAt( int i);
    int getReasonAt( int i);

private:
    std::vector<GOPElement> gopVector;

    Eigen::Matrix3f lastKeyFr_R;
    Eigen::Vector3f lastKeyFr_T;
};


#endif //__GOP_H___

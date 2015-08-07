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

#include <Eigen/Dense>


/// Stores the poses of each frame processed and provides an interface for GOP structure
class GOP
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GOP();

    void setAsOrdinaryFrame(int frameNum, Eigen::Matrix3f cR, Eigen::Vector3f cT );
    void setAsKeyFrame(int frameNum, int reason, Eigen::Matrix3f cR, Eigen::Vector3f cT );

private:
    bool keyFrame;
    int frameId;
    int reason_of_change; //this is valid only if isKeyFrame true

    Eigen::Matrix3f nowRel_R; //relative to prev-reference frame
    Eigen::Vector3f nowRel_T;

    Eigen::Matrix3f world_R; //in world-ordinate system (ie. co-ordinate system of 1st frame
    Eigen::Vector3f world_T;

    Eigen::Matrix3f lastKeyFrame_R;
    Eigen::Vector3f lastKeyFrame_T;

};




#endif //__GOP_H___

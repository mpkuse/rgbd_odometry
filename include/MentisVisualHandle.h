#ifndef __MentisVisualHandle_H___
#define __MentisVisualHandle_H___

///////////////////////////////////////////////////////////////////////////
//                        MentisVisualHandle                             //
///////////////////////////////////////////////////////////////////////////
//                                                                       //
//  Author : Manohar Kuse <mpkuse@ust.hk>                                //
//  Created on : 22nd July, 2015                                         //
//  Purpose : Defines a class for visualization to RViz.                 //
///////////////////////////////////////////////////////////////////////////




#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <Eigen/Dense>

class SolveDVO; //forward declaration of a friend class

/// @class Defines a class for visualization to RViz.
class MentisVisualHandle
{
public:
    MentisVisualHandle();
    void setNodeHandle( SolveDVO* const dvoH );
    void setRVizFrameID( const char * frameID );

    void incrementalSphere();
    void publishCurrentPointCloud( int level );
    void publishPoseFinal(Eigen::Matrix3f &rot, Eigen::Vector3f &tran);
    void publishPath();



private:

    //
    // Node Handle (received from smewhr, eg SolveDVO class)
    ros::NodeHandle nh;
    SolveDVO* dvoHandle;
    bool isNodeHandleValid;

    //
    // All Publishers
    char const * rviz_frame_id; ///< RViz frameID
    ros::Publisher pub_inc_sp; ///< incremental sphere (testing)

    ros::Publisher pub_pc; ///< point-cloud
    ros::Publisher pub_final_pose; ///< final pose relative to first frame (ie. global frame)
    ros::Publisher pub_pose_wrt_ref; ///< publish pose with respect to ref frame
    ros::Publisher pub_path; ///< publish path (as evaluated by DVO)


    //
    // Past Pose Data
    std::vector<geometry_msgs::PoseStamped> poseAry;


    //helpers
    void matrixToPose(Eigen::Matrix3f rot, Eigen::Vector3f tran, geometry_msgs::Pose& rospose);

};


#endif //__MentisVisualHandle_H___

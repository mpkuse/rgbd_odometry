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
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>


#include <Eigen/Dense>
#include <igl/repmat.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

class SolveDVO; //forward declaration of a friend class

#define __DEBUG_FRAME_MARKER__ //visualize sphere at poses which were keyframes
//#define __DEBUG_FRAME_MARKER_ALL_FRAMES


/// @class Defines a class for visualization to RViz.
class MentisVisualHandle
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MentisVisualHandle();
    void setNodeHandle( SolveDVO* const dvoH );
    void setRVizFrameID( const char * frameID );

    void incrementalSphere();
    void publishCurrentPointCloud( int level );
    void publishPoseFinal(Eigen::Matrix3f &rot, Eigen::Vector3f &tran);
    void publishPath();

    // Reprojection debugging code
    void debug( Eigen::Matrix3f cR, Eigen::Vector3f cT );

    // Publishing from GOP
    void publishGOP();

    // Full Point Cloud (This is being a bit ambitious)
    void publishFullPointCloud();

    void publishFromTF(Eigen::Matrix3f &rot, Eigen::Vector3f &tran);

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
    ros::Publisher pub_global_pose; ///< final pose relative to first frame (ie. global frame)
    ros::Publisher pub_path; ///< publish path (as evaluated by DVO)

    ros::Publisher pub_gt_pose; ///< ground-truth pose
    ros::Publisher pub_gt_path; ///< ground-truth path

    //debug publishers
    ros::Publisher pub_debug_keyFrames; ///< publish keyFrame-locations as spheres

    //
    // Past Pose Data
    std::vector<geometry_msgs::PoseStamped> poseAry;

    //
    // All GT pose data
    std::vector<geometry_msgs::PoseStamped> gtPoseAry;


    //helpers
    void matrixToPose(Eigen::Matrix3f rot, Eigen::Vector3f tran, geometry_msgs::Pose& rospose);



    // Global Point cloud related
    sensor_msgs::PointCloud pcl_msg;
    sensor_msgs::ChannelFloat32 all_intensities;

};


#endif //__MentisVisualHandle_H___

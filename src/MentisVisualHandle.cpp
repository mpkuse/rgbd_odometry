///////////////////////////////////////////////////////////////////////////
//                        MentisVisualHandle                             //
///////////////////////////////////////////////////////////////////////////
//                                                                       //
//  Author : Manohar Kuse <mpkuse@ust.hk>                                //
//  Created on : 22nd July, 2015                                         //
//  Purpose : Defines a class for visualization to RViz.                 //
///////////////////////////////////////////////////////////////////////////


#include <MentisVisualHandle.h>
#include <SolveDVO.h>



MentisVisualHandle::MentisVisualHandle()
{
    isNodeHandleValid = false;
    rviz_frame_id = "denseVO"; //a default frameID

    poseAry.reserve(2000);
    poseAry.clear();
}

void MentisVisualHandle::setNodeHandle(SolveDVO *const dvoH )
{
    //
    /// Setup node handle <br/>
    this->dvoHandle = dvoH;
    isNodeHandleValid = true;


    //
    /// Setup publishers <br/>
    pub_inc_sp = dvoHandle->nh.advertise<visualization_msgs::Marker>( "/testVis/sphere", 100 );

    pub_pc = dvoHandle->nh.advertise<sensor_msgs::PointCloud>( "dvo/pointCloud", 1 );
    pub_final_pose = dvoHandle->nh.advertise<geometry_msgs::PoseStamped>( "dvo/finalPose", 1 );
    pub_pose_wrt_ref = dvoHandle->nh.advertise<geometry_msgs::PoseStamped>( "dvo/poseWrtRef", 1 );
    pub_path = dvoHandle->nh.advertise<nav_msgs::Path>("/dvo/path", 1 );

}


/// Set frameID. This is not a compulsary call as the default constructor sets a default string (denseVO)
void MentisVisualHandle::setRVizFrameID(const char *frameID)
{
    this->rviz_frame_id = frameID;
}

void MentisVisualHandle::incrementalSphere()
{

    ROS_INFO( "Publish a sphere........frameID : %s", rviz_frame_id );

    visualization_msgs::Marker marker;
    marker.header.frame_id = rviz_frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;


    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;

    marker.scale.x = 21.0;
    marker.scale.y = 21.0;
    marker.scale.z = 12.0;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    pub_inc_sp.publish(marker);
}



void MentisVisualHandle::publishCurrentPointCloud( int level )
{
    assert( dvoHandle->isCameraIntrinsicsAvailable && dvoHandle->isNowFrameAvailable );
    assert( dvoHandle->im_n.size() > 1 );


    sensor_msgs::PointCloud pcl_msg;
    pcl_msg.header.frame_id = rviz_frame_id;
    pcl_msg.header.stamp = ros::Time::now();


    float scaleFac = (float)pow(2,-level);
    Eigen::MatrixXf& _im = dvoHandle->im_n[level];
    Eigen::MatrixXf& _dim = dvoHandle->dim_n[level];

    sensor_msgs::ChannelFloat32 shade;
    shade.name = "intensity";


    for( int yy=0 ; yy<_im.rows() ; yy++ )
    {
        for( int xx=0 ; xx<_im.cols() ; xx++ )
        {
            float Z = _dim(yy,xx);
            if( Z < 10 )
                continue;
            float X = Z * (xx-scaleFac*dvoHandle->cx) / (scaleFac*dvoHandle->fx);
            float Y = Z * (yy-scaleFac*dvoHandle->cy) / (scaleFac*dvoHandle->fy);

            geometry_msgs::Point32 pt;
            pt.x = X;
            pt.y = Y;
            pt.z = Z;

            ROS_INFO_ONCE( "%.4f %.4f %.4f", X, Y, Z );

            pcl_msg.points.push_back(pt);
            shade.values.push_back( _im(yy,xx) );
        }
    }

    pcl_msg.channels.push_back(shade);
    pub_pc.publish( pcl_msg );
}




/// @brief Publishes the pose of kth frame with respect to global frame (ie. 1st frame)
/// Publishes to var `pub_final_pose`
void MentisVisualHandle::publishPoseFinal(Eigen::Matrix3f &rot, Eigen::Vector3f &tran)
{
    geometry_msgs::Pose rospose;
    matrixToPose(rot, tran, rospose);

    ROS_INFO( "Publishing Pose : [ %.4f %.4f %.4f %.4f :: %.4f %.4f %.4f", rospose.orientation.x, rospose.orientation.y, rospose.orientation.z, rospose.orientation.w,
                     rospose.position.x, rospose.position.y, rospose.position.z );

    geometry_msgs::PoseStamped poseS;
    poseS.header.frame_id = rviz_frame_id;
    poseS.header.stamp = ros::Time::now();
    poseS.pose = rospose;
    poseAry.push_back(poseS);

    pub_final_pose.publish( poseS );

}

void MentisVisualHandle::publishPath()
{
    nav_msgs::Path pathMsg;
    pathMsg.header.frame_id = rviz_frame_id;
    pathMsg.header.stamp = ros::Time::now();
    for( int i=0 ; i<poseAry.size() ; i++ )
        pathMsg.poses.push_back(poseAry[i]);
    pub_path.publish(pathMsg);
}



/// @brief Given the rotation and translation matrix convert to ros Pose representation
/// @param[in] rot : 3x3 rotation matrix
/// @param[in] trans : 3-vector representing translation
/// @param[out] rosPose : geometry_msgs::Pose as output
void MentisVisualHandle::matrixToPose(Eigen::Matrix3f rot, Eigen::Vector3f tran, geometry_msgs::Pose& rospose)
{
    Eigen::Quaternionf quat(rot);



    rospose.position.x = 1000.*tran(0);
    rospose.position.y = 1000.*tran(1);
    rospose.position.z = 1000.*tran(2);
    rospose.orientation.x = quat.x();
    rospose.orientation.y = quat.y();
    rospose.orientation.z = quat.z();
    rospose.orientation.w = quat.w();
}

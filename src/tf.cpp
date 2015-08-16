/*
 *                  Playing around with /tf/tfMessages
 *
 * Author : Manohar Kuse <mpkuse@ust.hk>
 * Date   : 14th Aug, 2015
 */




#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <Eigen/Dense>



char const * rviz_frame_id = "denseVO";



void tfTransform2EigenMat( tf::StampedTransform& tr, Eigen::Matrix3f& R, Eigen::Vector3f& T )
{
    Eigen::Quaternionf Qf(tr.getRotation().getW(), tr.getRotation().getX(), tr.getRotation().getY(), tr.getRotation().getZ() );
    R = Qf.toRotationMatrix();
    T(0) = tr.getOrigin().getX();
    T(1) = tr.getOrigin().getY();
    T(2) = tr.getOrigin().getZ();
}

void tfTransform2Pose(tf::StampedTransform& transform, geometry_msgs::Pose& pose )
{
    tf::Quaternion quat = transform.getRotation();
    tf::Vector3 trans = transform.getOrigin();
    pose.orientation.x = (float) quat.getX();
    pose.orientation.y = (float) quat.getY();
    pose.orientation.z = (float) quat.getZ();
    pose.orientation.w = (float) quat.getW();
    pose.position.x    = (float) trans.getX();
    pose.position.y    = (float) trans.getY();
    pose.position.z    = (float) trans.getZ();
}


/// @brief Given the rotation and translation matrix convert to ros Pose representation
/// @param[in] rot : 3x3 rotation matrix
/// @param[in] trans : 3-vector representing translation
/// @param[out] rosPose : geometry_msgs::Pose as output
void matrixToPose(Eigen::Matrix3f& rot, Eigen::Vector3f& tran, geometry_msgs::Pose& rospose)
{
    Eigen::Quaternionf quat(rot);

    rospose.position.x = tran(0);
    rospose.position.y = tran(1);
    rospose.position.z = tran(2);
    rospose.orientation.x = quat.x();
    rospose.orientation.y = quat.y();
    rospose.orientation.z = quat.z();
    rospose.orientation.w = quat.w();
}


int main( int argc, char ** argv )
{
    ros::init(argc, argv, "tf_play");
    ros::NodeHandle nh;


    tf::TransformListener listener;
    ros::Publisher pub_gt_pose = nh.advertise<geometry_msgs::PoseStamped>( "/dvo/GTpose", 10 );
    ros::Publisher pub_gt_path = nh.advertise<nav_msgs::Path>("/dvo/GTpath", 10 );

    nav_msgs::Path pathMsg;
    pathMsg.header.frame_id = rviz_frame_id;


    ros::Rate rate(30);
    int loopCount = 0;


    Eigen::Matrix3f Rf, Rc, Rc_f;
    Eigen::Vector3f Tf, Tc, Tc_f;

    while( ros::ok() )
    {
        tf::StampedTransform transform;

        try {
        listener.lookupTransform( "/world", "/openni_rgb_optical_frame", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }


        if( loopCount == 0 )
        {
            //first pose
            tfTransform2EigenMat(transform, Rf, Tf);
        }

        ROS_INFO_THROTTLE( 10, "Receiving tf data" );

        // convert to geometry_msgs/PoseStampted
        geometry_msgs::Pose pose;

        tfTransform2EigenMat(transform, Rc, Tc);
        Tc_f = Rf.transpose() * ( Tc - Tf );
        Rc_f = Rf.transpose() * Rc;

        matrixToPose( Rc_f, Tc_f, pose );

        geometry_msgs::PoseStamped poseMsg;
        poseMsg.header.stamp = ros::Time::now(); //data[0];
        poseMsg.header.frame_id = rviz_frame_id;
        poseMsg.pose = pose;

        pathMsg.poses.push_back(poseMsg);
        pathMsg.header.stamp = ros::Time::now();

        pub_gt_pose.publish( poseMsg );
        pub_gt_path.publish( pathMsg );



        ros::spinOnce();
        rate.sleep();
        loopCount++;
    }
    ros::spin();
}

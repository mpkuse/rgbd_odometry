#include <ros/ros.h>

#include <math.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <sophus/se3.hpp>
#include <Eigen/Geometry>


ros::Publisher pub;


/// @brief Given the rotation and translation matrix convert to ros Pose representation
/// @param[in] rot : 3x3 rotation matrix
/// @param[in] trans : 3-vector representing translation
/// @param[out] rosPose : geometry_msgs::Pose as output
void matrixToPose(Eigen::Matrix3f rot, Eigen::Vector3f tran, geometry_msgs::Pose& rospose)
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


/// Publish pose from Euler angles
void publishPose(Eigen::MatrixXf rot, Eigen::VectorXf tran)
{
    geometry_msgs::Pose rospose;
    matrixToPose(rot, tran,rospose );


    geometry_msgs::PoseStamped poseS;
    poseS.header.frame_id = "denseVO";
    poseS.header.stamp = ros::Time::now();
    poseS.pose = rospose;

    pub.publish( poseS );

}


void imuDataRcvd( const sensor_msgs::Imu& msg )
{
    //ROS_INFO( "Linear acc : %f %f %f", msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z );
    float Rx = msg.linear_acceleration.x;
    float Ry = msg.linear_acceleration.y;
    float Rz = msg.linear_acceleration.z;
    float R  = sqrt( Rx*Rx + Ry*Ry + Rz*Rz );

    ROS_INFO( "Linear acc : %f %f %f",  (float)acos(Rx/R)*180/M_PI,  (float)acos(Ry/R)*180/M_PI,  (float)acos(Rz/R)*180/M_PI );
    Eigen::Matrix3f Rot;
    Rot = Eigen::AngleAxisf(  (float)acos(Rx/R), Eigen::Vector3f::UnitX()) *
                          Eigen::AngleAxisf( (float)acos(Ry/R), Eigen::Vector3f::UnitZ()) *
                          Eigen::AngleAxisf(  (float)acos(Rz/R), Eigen::Vector3f::UnitY());


    Rot.transposeInPlace();


    Eigen::Vector3f tr(100,100,100);
    publishPose( Rot, tr );

}


/*
int main( int argc, char ** argv )
{
    ros::init(argc, argv, "imuTest_node");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::PoseStamped>( "imu/pose", 1 );
    ros::Subscriber sub = nh.subscribe( "/imu_3dm_gx4/imu", 100, imuDataRcvd );

    ros::spin();

}
*/


// testing sophus SE3 lib
int main( int argc, char ** argv )
{
    Eigen::MatrixXf psi = Eigen::VectorXf::Zero(6);
    psi <<  7.42664e-06, -4.29979e-06, -3.42235e-05, -4.42348e-06,  7.43644e-06, -2.34198e-05;
    //Eigen::Matrix<float,6,1> psi;
    //psi = Eigen::VectorXf::Zero(6);
    //Sophus::SE3f mat = Sophus::exp( psi );
    Sophus::SE3f mat = Sophus::SE3f::exp(psi);
    std::cout << mat.rotationMatrix();
    Eigen::Quaternionf quat( mat.rotationMatrix() );
    std::cout << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << "\n" ;
}

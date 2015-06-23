/**
  SolvePnP.h

  Author : Manohar Kuse
  Date   : 3rd May, 2015

  Defines the class to handle streaming images of the chess board and compute 3d-2d PnP
  */


#ifndef ___IMU_DEAD_RECKON___
#define ___IMU_DEAD_RECKON___

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>




/// Defines the class to receive IMU data and perform dead-reckoning navigation from IMU
class ImuDeadReckon
{
public:
    ImuDeadReckon();
    void ImuSetIntrinsics(float accelerometer_noise_var, float accelerometer_bias, float gyroscope_noise_var, float gyroscope_bias);
    void eventLoop();

private:
    // Publishers and Subscribers
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub_dr_pose; ///< Dead reckoning pose
    char const * rviz_frame_id;



    // IMU Intrinsics
    float a_b, a_var; ///< accelerometer bias and variance
    Eigen::Vector3f aBias;
    float g_b, g_var; ///< gyroscope bias and variance
    Eigen::Vector3f gBias;

    Eigen::Vector3f gravity;
    bool isIMUIntrinsicsReady;

    // Current IMU Data
    Eigen::Vector3f lin_acc; ///< Linear Accleration
    Eigen::Vector3f ang_vel; ///< Angular velocity
    bool isIMUDataReady;


    // Callback
    void imuDataRcvd( const sensor_msgs::Imu& msg );


    // Nominal State Variables
    Eigen::Vector3f nsv_p; ///< Position
    Eigen::Vector3f nsv_v; ///< Velocity
    Eigen::Quaternionf nsv_q; ///< Quaternion
    float delT;

    // updates
    void updateNominalStateWithCurrentMeasurements();

    //helpers
    void makeQuaternionFromVector(Eigen::Vector3f &inVec, Eigen::Quaternionf &outQuat  );


    // Pusblishers routines
    void publishPose();


};





#endif //___IMU_DEAD_RECKON___

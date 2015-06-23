#include <ImuDeadReckon.h>


/// default constructor. Init the subscribers
ImuDeadReckon::ImuDeadReckon()
{
    sub = nh.subscribe( "imu_3dm_gx4/imu", 2, &ImuDeadReckon::imuDataRcvd, this );
    pub_dr_pose = nh.advertise<geometry_msgs::PoseStamped>( "/imu/pose", 1 );

    // Global Constants
    rviz_frame_id = "denseVO";


    //init nominal state variables
    nsv_p = Eigen::Vector3f::Zero();
    nsv_v = Eigen::Vector3f::Zero();
    nsv_q.vec() = Eigen::Vector3f::Zero();
    nsv_q.w() = 1.0f;
    delT = 1/100.0f; //100 hz

    gravity(0) = 0.0f;
    gravity(1) = 9.7874f;
    gravity(2) = 0.0;

    isIMUIntrinsicsReady = false;
    isIMUDataReady = false;

}

void ImuDeadReckon::ImuSetIntrinsics(float accelerometer_noise_var, float accelerometer_bias, float gyroscope_noise_var, float gyroscope_bias)
{
    this->a_var = accelerometer_noise_var;
    this->a_b = accelerometer_bias;

    this->g_b = gyroscope_bias;
    this->g_var = gyroscope_noise_var;

    aBias = a_b*Eigen::Vector3f::Ones();
    gBias = g_b*Eigen::Vector3f::Ones();

    isIMUIntrinsicsReady = true;
}





/// @brief Callback for IMU data received. Better be very fast. data arrive at 100Hz
void ImuDeadReckon::imuDataRcvd( const sensor_msgs::Imu& msg )
{
    isIMUDataReady = false;
    //ROS_INFO( "Linear acc : %f %f %f", msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z );
    lin_acc(0) = msg.linear_acceleration.x;
    lin_acc(1) = msg.linear_acceleration.y;
    lin_acc(2) = msg.linear_acceleration.z;


    ang_vel(0) = msg.angular_velocity.x;
    ang_vel(1) = msg.angular_velocity.y;
    ang_vel(2) = msg.angular_velocity.z;


    isIMUDataReady = true;

}

void ImuDeadReckon::updateNominalStateWithCurrentMeasurements()
{
    assert( isIMUIntrinsicsReady && isIMUDataReady );

    Eigen::Matrix3f R = nsv_q.toRotationMatrix();
    Eigen::Vector3f aBiasCorrected = lin_acc - aBias ;
    Eigen::Vector3f gBiasCorrected = (ang_vel - gBias)*delT ;


    nsv_p += nsv_v*delT + 0.5*( R*aBiasCorrected + gravity )*delT*delT;
    nsv_v += (R*aBiasCorrected + gravity) * delT;

    Eigen::Quaternionf Q;
    makeQuaternionFromVector( gBiasCorrected, Q );
    nsv_q = nsv_q * ( Q ); //quaternion multiply

}

void ImuDeadReckon::makeQuaternionFromVector( Eigen::Vector3f& inVec, Eigen::Quaternionf& outQuat )
{
    float phi = inVec.norm();
    Eigen::Vector3f u = inVec / phi; // u is a unit vector

    outQuat.vec() = u * sin( phi / 2.0 );
    outQuat.w()   =     cos( phi / 2.0 );


    //outQuat = Eigen::Quaternionf( 1.0, 0., 0., 0. );
}



/// @brief The event loop. Basically an ever running while with ros::spinOnce()
/// This is a re-implementation taking into care the memory scopes and also processing only points with higher image gradients
void ImuDeadReckon::eventLoop()
{
    ros::Rate rate(100); //100Hz
    while( ros::ok() )
    {
        ros::spinOnce();

        if( !(this->isIMUDataReady) )
            continue;


        updateNominalStateWithCurrentMeasurements();
        publishPose();


        ROS_INFO_STREAM_THROTTLE( 5, "(every5 sec) Lin Acc : "<< lin_acc.transpose() );
        ROS_INFO_STREAM_THROTTLE( 5, "Ang Vel : "<< ang_vel.transpose() );

        rate.sleep();
    }

}



/// @brief Publishes the current nominal-state-pose
void ImuDeadReckon::publishPose()
{
    geometry_msgs::Pose rospose;
    rospose.position.x = nsv_p(0);
    rospose.position.y = nsv_p(1);
    rospose.position.z = nsv_p(2);
    rospose.orientation.w = nsv_q.w();
    rospose.orientation.x = nsv_q.x();
    rospose.orientation.y = nsv_q.y();
    rospose.orientation.z = nsv_q.z();

    geometry_msgs::PoseStamped poseS;
    poseS.header.frame_id = rviz_frame_id;
    poseS.header.stamp = ros::Time::now();
    poseS.pose = rospose;

    pub_dr_pose.publish( poseS );

}

#include <ImuDeadReckon.h>


/// default constructor. Init the subscribers
ImuDeadReckon::ImuDeadReckon()
{
    sub = nh.subscribe( "imu_3dm_gx4/imu", 2, &ImuDeadReckon::imuDataRcvd, this );

    isIMUIntrinsicsReady = false;
    isIMUDataReady = false;

}

void ImuDeadReckon::ImuSetIntrinsics(float accelerometer_noise_var, float accelerometer_bias, float gyroscope_noise_var, float gyroscope_bias)
{
    this->a_var = accelerometer_noise_var;
    this->a_b = accelerometer_bias;

    this->g_b = gyroscope_bias;
    this->g_var = gyroscope_noise_var;

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

        ROS_INFO_STREAM( "Lin Acc : "<< lin_acc.transpose() );
        ROS_INFO_STREAM( "Ang Vel : "<< ang_vel.transpose() );

        rate.sleep();
    }

}

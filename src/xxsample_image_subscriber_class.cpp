#include <SolvePnP.h>


/// default constructor. Init the subscribers
SolvePnP::SolvePnP()
{
    sub = nh.subscribe( "odometry/rgbd", 2, &SolvePnP::imageArrivedCallBack, this );

}




/// @brief Callback to receive RGBD from the topic "odometry/rgbd"
///
/// The subscriber is defined in the construction. This function is the callback for the same.
/// This is a blocking function. Thus do not do any processing here.
void SolvePnP::imageArrivedCallBack( rgbd_odometry::RGBDFrameConstPtr msg )
{
    ROS_INFO_STREAM_ONCE( "1st RGBD frame received. Will continue receiving but not report anymore on this");
    isFrameAvailable=false;


    cv::Mat frame, dframe;
    try
    {
        frame =  cv_bridge::toCvShare(  msg->frame, msg, "bgr8" )->image ;
        dframe =  cv_bridge::toCvShare(  msg->dframe, msg, "mono16" )->image ;
    }
    catch( cv_bridge::Exception& e )
    {
        ROS_ERROR( "cv_bridge exception: %s", e.what() );
        isFrameAvailable = false;
        return;
    }

    dframe.setTo(1, (dframe==0) ); //to avoid zero depth

    frame.copyTo( this->rcvd_frame );
    dframe.copyTo( this->rcvd_dframe );
    isFrameAvailable=true;

}


/// @brief The event loop. Basically an ever running while with ros::spinOnce()
/// This is a re-implementation taking into care the memory scopes and also processing only points with higher image gradients
void SolvePnP::eventLoop()
{
    while( ros::ok() )
    {
        ros::spinOnce();
        if( !(this->isFrameAvailable) )
            continue;

        cv::imshow( "im", this->rcvd_frame );
        cv::waitKey(3);
    }

}

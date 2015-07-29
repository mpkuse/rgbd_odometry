/// @file A simple point based odometry. 3d-2d PNP

#include <PnPOdometry.h>

int main( int argc, char ** argv )
{
    ros::init(argc, argv, "pnp_odometry");
    PnPOdometry odo;
    odo.setCameraMatrix( "Freiburg_ROS_default_320x240.xml" );

    odo.eventLoop();


    return 0;
}

#include <ros/ros.h>

#include <SolvePnP.h>




int main( int argc, char ** argv )
{
    ros::init(argc, argv, "pnp_node");
    SolvePnP pnp;
    pnp.eventLoop();

}

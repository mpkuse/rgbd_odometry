#include <ros/ros.h>

#include <SolveDVO.h>




int main( int argc, char ** argv )
{
    ros::init(argc, argv, "dvo_node");
    SolveDVO dvo;
    //dvo.setCameraMatrix( "params_mm_scale_half.xml" ); //w/o distortion coifs
    //dvo.setCameraMatrix( "Xtion_320x240.xml" );
    dvo.setCameraMatrix( "Freiburg_ROS_default_320x240.xml" );
    dvo.loop();

}

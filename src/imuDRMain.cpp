#include <ImuDeadReckon.h>


int main( int argc, char ** argv)
{
    ros::init(argc, argv, "ImuDeadReckon_node");
    ImuDeadReckon dr;
    dr.ImuSetIntrinsics(  1.0e-03,  0.039e-02,  8.73e-05,  4.8e-05   );
    dr.eventLoop();
}

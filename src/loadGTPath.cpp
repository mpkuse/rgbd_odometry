#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

char const * rviz_frame_id = "denseVO";

int main( int argc, char ** argv )
{
    //
    /// Check for valid usage
    if( argc != 2)
    {
        ROS_ERROR( "Invalid Usage....Expected 1 argument (name of GroundTruth Pose file)" );
        ROS_ERROR( "Usage :: %s <fileName>", argv[0] );
        ROS_INFO( "The input file : 'timestamp tx ty tz qx qy qz qw'" );
        exit(1);
    }

    //
    /// Attempt to open data file eg. TUM_RGBD/rgbd_dataset_freiburg1_rpy-groundtruth.txt
    ROS_INFO( "Attempting to open '%s'", argv[1] );
    ifstream fin(argv[1]);
    if( fin.is_open() == false )
    {
        ROS_ERROR( "Cannot open file '%s'", argv[1] );
        exit(1);
    }


    //
    /// ROS Init
    ros::init( argc, argv, "publishGTPath" );
    ros::NodeHandle nh;


    ros::Publisher pub = nh.advertise<nav_msgs::Path>("/dvo/GTpath", 1 );
    ros::Publisher pub_pose = nh.advertise<geometry_msgs::PoseStamped>( "/dvo/GTpose", 1 );



    string line;
    int lineCount = 0;

    //
    /// nav_msgs::Path
    nav_msgs::Path pathMsg;
    pathMsg.header.frame_id = rviz_frame_id;
    pathMsg.header.stamp = ros::Time::now();

    ros::Rate rate(100);
    while( getline(fin, line) && ros::ok() )
    {
        //the following line trims white space from the beginning of the string
        line.erase(line.begin(), find_if(line.begin(), line.end(), not1(ptr_fun<int, int>(isspace))));

        if(line[0] == '#') continue;

        float data[10];

        stringstream ss(line);

        for( int i=0 ; i<8 ; i++ )  {
            ss >> data[i];
        }

        geometry_msgs::PoseStamped poseMsg;
        poseMsg.header.stamp = ros::Time::now(); //data[0];
        poseMsg.header.frame_id = rviz_frame_id;
        poseMsg.pose.position.x = data[1];
        poseMsg.pose.position.y = data[2];
        poseMsg.pose.position.z = data[3];
        poseMsg.pose.orientation.x = data[4];
        poseMsg.pose.orientation.y = data[5];
        poseMsg.pose.orientation.z = data[6];
        poseMsg.pose.orientation.w = data[7];

        pub_pose.publish( poseMsg );

        pathMsg.poses.push_back(poseMsg);


//        cout << "Data: " << data[5]  << endl;
//        cout << "Data: " << line  << endl;


        lineCount++;



        ros::spinOnce();
        rate.sleep();
    }

    pub.publish( pathMsg );
    return 0;
}

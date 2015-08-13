#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>

using namespace std;

char const * rviz_frame_id = "denseVO";


void parseData( string& line, std::vector<float>& parsedData )
{
    stringstream ss(line);

    parsedData.clear();

    for( int i=0 ; i<8 ; i++ )  {
        float tmp;
        ss >> tmp;
        parsedData.push_back(tmp);
    }
}


/// @brief Given the rotation and translation matrix convert to ros Pose representation
/// @param[in] rot : 3x3 rotation matrix
/// @param[in] trans : 3-vector representing translation
/// @param[out] rosPose : geometry_msgs::Pose as output
void matrixToPose(Eigen::Matrix3f& rot, Eigen::Vector3f& tran, geometry_msgs::Pose& rospose)
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


    std::vector<float> ln;
    Eigen::Matrix3f Rf;  Eigen::Vector3f Tf; //rotation and translation of 1st frame (in world cord-system)
    Eigen::Matrix3f Rc;  Eigen::Vector3f Tc; //rotation and translation of current frame (in world cord-system)
    Eigen::Matrix3f Ru;  Eigen::Vector3f Tu; //rotation and translation of current frame (in frame of 1st frame)



    ros::Rate rate(100);
    //skip 1st 10 lines
    int tmpCount=0;
    while( getline(fin, line) && ros::ok() )
    {
        line.erase(line.begin(), find_if(line.begin(), line.end(), not1(ptr_fun<int, int>(isspace))));

        if(line[0] == '#') continue;

        tmpCount++;

        if( tmpCount > 350 )
            break;
    }

    while( getline(fin, line) && ros::ok() )
    {
        //the following line trims white space from the beginning of the string
        line.erase(line.begin(), find_if(line.begin(), line.end(), not1(ptr_fun<int, int>(isspace))));

        if(line[0] == '#') continue;


        parseData( line, ln );

        if( lineCount == 0 ) //register the 1st line. this is pose of the 1st frame which is my world-frame
        {
            Eigen::Quaternionf Qf(ln[7], ln[4], ln[5], ln[6]);
            Rf = Qf.toRotationMatrix();
            Tf(0) = ln[1];
            Tf(1) = ln[2];
            Tf(2) = ln[3];
        }

        Eigen::Quaternionf Qc(ln[7], ln[4], ln[5], ln[6]);
        Rc = Qc.toRotationMatrix();
        Tc(0) = ln[1];
        Tc(1) = ln[2];
        Tc(2) = ln[3];


        // compute [R,T] in system of 1st frame
        Tu = Rf.transpose() * ( Tc - Tf );
        Ru = Rf.transpose() * Rc;



        // Get pose using Ru, Tu
        geometry_msgs::Pose poseU;
        matrixToPose( Ru, Tu, poseU );


        ROS_INFO( ".");




        geometry_msgs::PoseStamped poseMsg;
        poseMsg.header.stamp = ros::Time::now(); //data[0];
        poseMsg.header.frame_id = rviz_frame_id;
        poseMsg.pose = poseU;

        pub_pose.publish( poseMsg );

        pathMsg.poses.push_back(poseMsg);
        pathMsg.header.stamp = ros::Time::now();

        pub.publish( pathMsg );


//        cout << "Data: " << data[5]  << endl;
//        cout << "Data: " << line  << endl;


        lineCount++;



        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO( "Read %d lines", lineCount );

    return 0;
}

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt32.h>

#include <rgbd_odometry/RGBDFrame.h>


#include <PrimeSenseCam.h>



cv::Mat cameraMatrix, distCoeffs;
void setCameraMatrix(const char *calibFile)
{
    //
    // Read calibration matrix, distortion coiffs
    //
    cv::FileStorage fs(calibFile, cv::FileStorage::READ);
    if( fs.isOpened() == false )
    {
        ROS_ERROR_STREAM( "[setCameraMatrix] Error opening camera "
                "params file : "<< calibFile );
        return;
    }

    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    ROS_INFO( "Loaded Calib Successfully : %s", calibFile );
    ROS_INFO_STREAM( "cameraMatrix : \n"<< cameraMatrix );
    ROS_INFO_STREAM( "distortions : "<< distCoeffs );
}





void undistortFrame( cv::Mat& src, cv::Mat& dst )
{
    cv::Mat undistorted;
    cv::undistort(src, undistorted, cameraMatrix, distCoeffs );
    undistorted.copyTo(dst);
//    cv::imshow("src", src);
//    cv::imshow("undistorted", undistorted );
//    cv::waitKey(3);
}


void undistortDFrame( cv::Mat& src, cv::Mat& dst )
{
    cv::Mat undistorted;
    cv::undistort(src, undistorted, cameraMatrix, distCoeffs );
    undistorted.copyTo(dst);
//    cv::imshow("src_dframe", src);
//    cv::imshow("undistorted_dframe", undistorted );
//    cv::waitKey(3);
}


int main( int argc, char ** argv )
{
    setCameraMatrix("Xtion_640x480.xml");


    ros::init(argc, argv, "frame_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<rgbd_odometry::RGBDFrame>("odometry/rgbd", 2 );

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_im = it.advertise( "odometry/image", 1);






    //cv::Mat image = cv::imread( "Lenna.png", CV_LOAD_IMAGE_COLOR );


    ros::Rate rate(30);

    PrimeSenseCam cam;
    cam.start();

    cv::Mat frame, dframe;
    cv::Mat tmpf, tmpd;
    cv::Mat orgf, orgd;
    cv::Mat frame_gray;
    cam.retriveFrame(orgf,orgd);


    //camera_info. Only publishing K and image dimensions
    ros::Publisher pub_cam_info = nh.advertise<sensor_msgs::CameraInfo>("odometry/cam_info", 2 );
    sensor_msgs::CameraInfo infoMsg;
    infoMsg.width = .5*orgf.cols;
    infoMsg.height = .5*orgf.rows;
    infoMsg.K[0] = .5*cameraMatrix.at<double>(0,0);
    infoMsg.K[1] = 0.0;
    infoMsg.K[2] = .5*cameraMatrix.at<double>(0,2);
    infoMsg.K[3] = 0.0;
    infoMsg.K[4] = .5*cameraMatrix.at<double>(1,1);
    infoMsg.K[5] = .5*cameraMatrix.at<double>(1,2);
    infoMsg.K[6] = 0.0;
    infoMsg.K[7] = 0.0;
    infoMsg.K[8] = 1.0;

    infoMsg.P[0] = .5*cameraMatrix.at<double>(0,0);
    infoMsg.P[1] = 0.0;
    infoMsg.P[2] = .5*cameraMatrix.at<double>(0,2);
    infoMsg.P[3] = 0.0; //
    infoMsg.P[4] = 0.0;
    infoMsg.P[5] = .5*cameraMatrix.at<double>(1,1);
    infoMsg.P[6] = .5*cameraMatrix.at<double>(1,2);
    infoMsg.P[7] = 0.0; //
    infoMsg.P[8] = 0.0;
    infoMsg.P[9] = 0.0;
    infoMsg.P[10] = 1.0;
    infoMsg.P[11] = 0.0;

    for( int i=0 ; i<9 ; i++ )
        cout<< infoMsg.P[i] << ", ";
    cout<< endl;


    while( ros::ok() )
    {
        ROS_INFO_STREAM_THROTTLE( 30, "Retriving @ 30 fps");


        cam.retriveFrame(orgf,orgd);
        orgd.setTo(1, (orgd==0) ); //to avoid zero depth


        undistortFrame(orgf, tmpf);
        undistortDFrame(orgd, tmpd);
        tmpd.setTo(1, (tmpd==0) ); //to avoid zero depth




        cv::resize(tmpf, frame, cv::Size(), .5, .5, cv::INTER_NEAREST );
        cv::resize(tmpd, dframe, cv::Size(),.5, .5, cv::INTER_NEAREST );
        cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);


        sensor_msgs::ImagePtr frame_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", frame_gray).toImageMsg();
        sensor_msgs::ImagePtr dframe_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", dframe).toImageMsg();

        infoMsg.header = frame_msg->header;


        rgbd_odometry::RGBDFrame msg;
        msg.age = 33.4;
        msg.name = "Kiya";
        msg.frame = *frame_msg;
        msg.dframe = *dframe_msg;

        pub.publish(msg);
        pub_im.publish( frame_msg );
        pub_cam_info.publish( infoMsg );

        rate.sleep();
    }

}

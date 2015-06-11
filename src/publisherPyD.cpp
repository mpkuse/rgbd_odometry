#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <rgbd_odometry/RGBDFrame.h>
#include <rgbd_odometry/RGBDFramePyd.h>


#include <PrimeSenseCam.h>


//#define __ENABLE_IM_WRITE__


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

///@file Publishes the Image Pyramid as `rgbd_odometry/RGBDFramePyd`
int main( int argc, char ** argv )
{

    //
    // Load camera matrix & distortions for undistort
    setCameraMatrix("Xtion_640x480.xml");
    cv::Mat newCameraMat = cv::Mat::zeros(3,3, cameraMatrix.type() );



    ros::init(argc, argv, "pyramidal_rgbd_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<rgbd_odometry::RGBDFramePyd>("Xtion/rgbdPyramid", 1 );

    //image_transport::ImageTransport it(nh);
    //image_transport::Publisher pub_im = it.advertise( "Xtion/image", 1);



    ros::Rate rate(30);

    PrimeSenseCam cam;
    cam.start();

    cv::Mat frame, dframe, framemono;
    cv::Mat tmpf, tmpd;
    cv::Mat orgf, orgd;

    int nFrame = 0;

    while( ros::ok() )
    {
        ROS_INFO_STREAM_THROTTLE( 30, "Retriving @ 30 fps");

        cam.retriveFrame(orgf,orgd);
        orgd.setTo(1, (orgd==0) ); //to avoid zero depth


        undistortFrame(orgf, tmpf);
        undistortDFrame(orgd, tmpd);
        tmpd.setTo(1, (tmpd==0) ); //to avoid zero depth



        rgbd_odometry::RGBDFramePyd msg;


#ifdef __ENABLE_IM_WRITE__
        /////////////////// FILE STORAGE ///////////////////
        char fName[100], imName[100];
        sprintf( fName, "xdump/framemono_%04d.xml", nFrame );
        sprintf( imName, "xdump/framemono_%04d.png", nFrame );
        ROS_INFO_STREAM_THROTTLE( 30, "Storing in File : "<< fName );


        cv::FileStorage fs(fName, cv::FileStorage::WRITE);


        ///////////////////////////////////////////////////
#endif //__ENABLE_IM_WRITE__


        float scale = 1.0;
        for( int i=0 ; i<4 ; i++ ) //at all Pyramidal level. Note level zero is 1/2, level-1 is 1/4 and so on
        {
            scale *= 0.5;

            cv::resize(tmpf, frame, cv::Size(), scale, scale, cv::INTER_NEAREST );
            cv::resize(tmpd, dframe, cv::Size(), scale, scale, cv::INTER_NEAREST );


            cv::cvtColor( frame, framemono, CV_BGR2GRAY );


#ifdef __ENABLE_IM_WRITE__
            /////////////////// FILE STORAGE ///////////////////


            if( i==0 )
                cv::imwrite( imName, framemono );
            char monoName[100], depthName[100];
            sprintf( monoName, "mono_%d", i );
            sprintf( depthName, "depth_%d", i );
            fs << monoName << framemono << depthName << dframe ;


            ///////////////////////////////////////////////////
#endif //__ENABLE_IM_WRITE__



            sensor_msgs::ImagePtr frame_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            sensor_msgs::ImagePtr framemono_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", framemono).toImageMsg();
            sensor_msgs::ImagePtr dframe_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", dframe).toImageMsg();


            msg.framergb.push_back( *frame_msg );
            msg.framemono.push_back( *framemono_msg );
            msg.dframe.push_back( *dframe_msg );

        }

#ifdef __ENABLE_IM_WRITE__
        fs.release();
#endif //__ENABLE_IM_WRITE__



        pub.publish(msg);
        //pub_im.publish( frame_msg );

        rate.sleep();
        nFrame++;
    }

}

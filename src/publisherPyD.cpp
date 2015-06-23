#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <rgbd_odometry/RGBDFrame.h>
#include <rgbd_odometry/RGBDFramePyd.h>


#include <PrimeSenseCam.h>


//#define __ENABLE_IM_WRITE__ //will also write a raw-video file
#define __ENABLE_VIDEO_WRITE__ //default is yuv
#define __FMT_Y4M__



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



string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}


/// @brief Writes the binary file with a rgbframe. Binary file is in .yuv (444) video format (Planar 444)
void frameWrite( std::fstream& fbin, cv::Mat& rgbFrame )
{
    assert( fbin.is_open() );
    assert( rgbFrame.rows > 0 && rgbFrame.cols > 0 );

    //convert frame to YUV
    cv::Mat yuv;
    cv::cvtColor( rgbFrame, yuv, CV_RGB2YUV );

    // yuv is of the type CV_8UC3
    ROS_INFO_STREAM_ONCE( "yuv.type : "<< type2str(yuv.type() ));


#ifdef __FMT_Y4M__
    //y4m each frame header
    char headerBuf[100];
    int charWr = sprintf( headerBuf, "FRAME%c",0x0a);
    fbin.write(headerBuf, charWr );
#endif


    std::vector<cv::Mat> mv;
    cv::split( yuv, mv);
    fbin.write(mv[0].data, mv[0].rows * mv[0].cols );
    fbin.write(mv[1].data, mv[0].rows * mv[0].cols );
    fbin.write(mv[2].data, mv[0].rows * mv[0].cols );

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

#ifdef __ENABLE_VIDEO_WRITE__
    //
    // Open Binary stream and write y4m header

    //for .yuv/y4m file (444 format)
    std::fstream fbin;

    char binFileName[150];

#ifdef __FMT_Y4M__
    sprintf( binFileName, "/home/eeuser/x265_trials/cam.y4m" );
#endif

#ifndef __FMT_Y4M__
    sprintf( binFileName, "/home/eeuser/x265_trials/cam.yuv" );
#endif



    fbin.open( binFileName, std::ios::binary| std::ios::out | std::ios::trunc );

    if( !fbin )
        ROS_ERROR( "Cannot Open video binary file %s", binFileName );
    else
        ROS_INFO( "Opened binary video file %s for writing", binFileName );


#ifdef __FMT_Y4M__
    // Write Y4M header
    char headerBuf[100];
    int charWr = sprintf( headerBuf, "YUV4MPEG2 ");
    assert( charWr == 10 );
    fbin.write(headerBuf, charWr );
    charWr = sprintf( headerBuf, "W%d H%d F30:1 C444%c", 320, 240, 0x0a );
    fbin.write(headerBuf, charWr );
#endif //__FMT_Y4M__

#endif //__ENABLE_VIDEO_WRITE__







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


#ifdef __ENABLE_VIDEO_WRITE__
            if( i==0 )
                frameWrite( fbin, frame );
#endif


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

#ifdef __ENABLE_VIDEO_WRITE__
        fbin.close();
#endif

}

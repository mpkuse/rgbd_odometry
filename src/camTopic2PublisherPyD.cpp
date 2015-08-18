/// @file Subscribes to camera messages (viz. camera_info, image, image_color). Publishes RGBDFramePyD message.
// Author : Manohar Kuse <mpkuse@ust.hk>
// Created on : 19th Jul, 2015


#include <ros/ros.h>
#include <fstream>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <rgbd_odometry/RGBDFramePyd.h>
#include <FColorMap.h>



//#define __ENABLE_DISPLAY
//#define __ENABLE_IM_WRITE__
//#define __ENABLE_VIDEO_WRITE__ //default is yuv
//#define __FMT_Y4M__
const char * outFolder = "xdump";


//#define __WRITE_CALIBRATION_DATA


//
// Subscriber Callbacks
cv::Mat rgb;
bool isRGBRcvd=false;

cv::Mat depth;
cv::Mat depth16;
bool isDepthRcvd=false;

cv::Mat cameraMatrix; ///< Camera matrix as available in rosbag
cv::Mat distCoeff;    ///< Distortion coeff as available in rosbag
bool isCameraParamsRcvd=false;

void depthCamInfoRcvd( const sensor_msgs::CameraInfo& msg )
{

}

void rgbCamInfoRcvd( const sensor_msgs::CameraInfo& msg )
{
    cameraMatrix = cv::Mat::zeros(3,3,CV_64F );
    cameraMatrix.at<double>(0,0) = (double)msg.K[0];
    cameraMatrix.at<double>(1,1) = (double)msg.K[4];
    cameraMatrix.at<double>(2,2) = 1.0;
    cameraMatrix.at<double>(0,2) = (double)msg.K[2];
    cameraMatrix.at<double>(1,2) = (double)msg.K[5];

    distCoeff = cv::Mat::zeros(5,1, CV_64F );
    for( int p=0 ; p<5 ; p++ )
        distCoeff.at<double>(p,0) = (double)msg.D[p];
    isCameraParamsRcvd=true;
}

void rgbRcvd( const sensor_msgs::ImageConstPtr& msg )
{
    isRGBRcvd = false;
    rgb = cv_bridge::toCvCopy(msg, "bgr8")->image;
    isRGBRcvd = true;
}

void depthRcvd( const sensor_msgs::ImageConstPtr& msg )
{
    isDepthRcvd = false;
    depth = cv_bridge::toCvCopy(msg)->image; //keep encoding as is, here it was 32FC1
    depth = 1000.0 * depth;
    depth.convertTo(depth16, CV_16UC1);
    depth16.setTo(1, (depth16==0) ); //to avoid zero depth
    isDepthRcvd = true;
}



//
// Distortion correction
void undistortFrame( cv::Mat& src, cv::Mat& dst )
{
    if( isCameraParamsRcvd == false )
    {
        ROS_ERROR_ONCE( "No Camera Params topic...not undistorting");
        src.copyTo(dst);
        return;
    }
    cv::Mat undistorted;
    cv::undistort(src, undistorted, cameraMatrix, distCoeff );
    undistorted.copyTo(dst);
    //    cv::imshow("src", src);
    //    cv::imshow("undistorted", undistorted );
    //    cv::waitKey(3);
}


void undistortDFrame( cv::Mat& src, cv::Mat& dst )
{
    if( isCameraParamsRcvd == false )
    {
        ROS_ERROR_ONCE( "No Camera Params topic...not undistorting");
        src.copyTo(dst);
        return;
    }
    cv::Mat undistorted;
    cv::undistort(src, undistorted, cameraMatrix, distCoeff );
    undistorted.copyTo(dst);
    //    cv::imshow("src_dframe", src);
    //    cv::imshow("undistorted_dframe", undistorted );
    //    cv::waitKey(3);
}




//
// Video out related


/// @brief Writes the binary file with a rgbframe. Binary file is in .yuv (444) video format (Planar 444)
void frameWrite( std::fstream& fbin, cv::Mat& rgbFrame )
{
    assert( fbin.is_open() );
    assert( rgbFrame.rows > 0 && rgbFrame.cols > 0 );

    //convert frame to YUV
    cv::Mat yuv;
    cv::cvtColor( rgbFrame, yuv, CV_RGB2YUV );

    // yuv is of the type CV_8UC3
    ROS_INFO_THROTTLE( 3, "Writing Video frames");



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




//
// DEBUGGING FUNCTIONS

std::string cvMatType2str(int type)
{
    std::string r;

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

void falseColorDepth( cv::Mat xdep )
{
    FColorMap fcm(64);

    cv::Mat falseColorMap = cv::Mat::zeros(xdep.rows, xdep.cols, CV_8UC3 );

    for( int c=0 ; c<xdep.cols ; c++ )
    {
        for( int r=0 ; r<xdep.rows ; r++ )
        {
            /*float mag = xdep.at<float>(r,c)/1000.0 ; //to take care of the applied scalling
            int colorIndx = 0;
            if( mag <= 0.5f )
                colorIndx = 0;
            if( mag > 0.5f && mag <= 1.0f)
                colorIndx = 10;
            else
                colorIndx = floor( mag*13 );
                */

            uint16_t mag = xdep.at<uint16_t>(r,c);
            int colorIndx;
            if( mag == 0 )
                colorIndx = 0;
            else if( mag < 1000 )
                colorIndx = 5;
            else
                colorIndx = (int) floor( ( (float)mag / 10000.0 ) * 50 );


            falseColorMap.at<cv::Vec3b>(r,c) = fcm.at(colorIndx);
        }
    }

    cv::imshow( "false color map of depth", falseColorMap);

}

int main( int argc, char ** argv )
{
    ros::init( argc, argv, "camTopic2PublisherPyD" );
    ros::NodeHandle nh;

    ros::Subscriber sub_depth_cam_info = nh.subscribe( "/camera/depth/camera_info", 2, depthCamInfoRcvd );
    ros::Subscriber sub_rgb_cam_info = nh.subscribe( "/camera/rgb/camera_info", 2, rgbCamInfoRcvd );

    ros::Subscriber sub_rgb_im = nh.subscribe( "/camera/rgb/image_color", 2, rgbRcvd );
    ros::Subscriber sub_depth_im = nh.subscribe( "/camera/depth/image", 2, depthRcvd );

    ros::Publisher pub = nh.advertise<rgbd_odometry::RGBDFramePyd>("Xtion/rgbdPyramid", 1 );




    cv::Mat frame, dframe, framemono;
    cv::Mat tmpf, tmpd;
    cv::Mat orgf, orgd;
    rgbd_odometry::RGBDFramePyd msg;


#ifdef __ENABLE_VIDEO_WRITE__
    //
    // Open Binary stream and write y4m header

    //for .yuv/y4m file (444 format)
    std::fstream fbin;

    char binFileName[150];

#ifdef __FMT_Y4M__
    sprintf( binFileName, "%s/cam.y4m", outFolder );
#endif

#ifndef __FMT_Y4M__
    sprintf( binFileName, "%s/cam.yuv", outFolder );
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







    ros::Rate rate(90); //Loop a little aggresively.
    int nFrame=0;
    while( nh.ok() )
    {
        ros::spinOnce();

        if( isRGBRcvd && isDepthRcvd ) {


            //
            // Undistort Images
            //     -- TO DO --
            cv::Mat rgb_undistorted, depth_undistorted;
            undistortFrame(rgb, rgb_undistorted);
            undistortDFrame(depth16, depth_undistorted);




            //
            // OpenXML file for writing
#ifdef __ENABLE_IM_WRITE__
            /////////////////// FILE STORAGE ///////////////////
            char fName[100], imName[100];
            sprintf( fName, "%s/framemono_%04d.xml", outFolder, nFrame );
            sprintf( imName, "%s/framemono_%04d.png", outFolder, nFrame );
            ROS_INFO_STREAM_THROTTLE( 30, "Storing in File : "<< fName );


            cv::FileStorage fs(fName, cv::FileStorage::WRITE);

            ///////////////////////////////////////////////////
#endif //__ENABLE_IM_WRITE__


            ROS_INFO_STREAM_THROTTLE( 5, "Processing Frame #"<< nFrame );


            msg.framergb.clear();
            msg.framemono.clear();
            msg.dframe.clear();

            //
            // For each pyd-level
            float scale = 1.0f;
            for( int i=0 ; i<4 ; i++ )
            {
                // note that, we do not want the max resolution ie. 640x480 is ignored (thtz too big for now)
                scale *= 0.5;

                cv::resize(rgb_undistorted, frame, cv::Size(), scale, scale, cv::INTER_NEAREST );
                cv::resize(depth_undistorted, dframe, cv::Size(), scale, scale, cv::INTER_NEAREST );

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

            pub.publish(msg);
            //unset the flags once this message is published. They will be activated again when new message arrives in the callback
            isRGBRcvd=isDepthRcvd=false;


#ifdef __ENABLE_DISPLAY
            cv::imshow( "rgb", rgb );
            //cv::imshow( "depth", depth );
            falseColorDepth(depth16);
#endif



            //            //
            //            // DEBUGGING
            //            {
            //                double minVal, maxVal;
            //                cv::minMaxIdx( depth16, &minVal, &maxVal );
            //                ROS_INFO_STREAM( "type : " << cvMatType2str(depth.type()) << " min :"<< minVal << " max :"<< maxVal );
            //                falseColorDepth(depth16);

            //                ROS_INFO_STREAM( nFrame );
            //                ROS_INFO( "fx:%.2lf  fy:%.2lf  cx:%.2lf  cy:%.2lf", cameraMatrix.at<double>(0,0), cameraMatrix.at<double>(1,1), cameraMatrix.at<double>(0,2), cameraMatrix.at<double>(1,2));
            //                ROS_INFO( "%.2lf, %.2lf, %.2lf, %.2lf, %.2lf", distCoeff.at<double>(0), distCoeff.at<double>(1), distCoeff.at<double>(2), distCoeff.at<double>(3), distCoeff.at<double>(4));
            //            }



#ifdef __ENABLE_IM_WRITE__
            fs.release();
#endif //__ENABLE_IM_WRITE__

            nFrame++;
            isRGBRcvd=false;
            isDepthRcvd=false;

        }

#ifdef __ENABLE_DISPLAY
        cv::waitKey(1);
#endif
        rate.sleep();
    }



#ifdef __ENABLE_VIDEO_WRITE__
        fbin.close();
#endif


#ifdef __WRITE_CALIBRATION_DATA
    //
    // Write calibration file
    if( isCameraParamsRcvd ) {

        ROS_INFO( "fx:%.2lf  fy:%.2lf  cx:%.2lf  cy:%.2lf", cameraMatrix.at<double>(0,0), cameraMatrix.at<double>(1,1), cameraMatrix.at<double>(0,2), cameraMatrix.at<double>(1,2));
        ROS_INFO( "%.2lf, %.2lf, %.2lf, %.2lf, %.2lf", distCoeff.at<double>(0), distCoeff.at<double>(1), distCoeff.at<double>(2), distCoeff.at<double>(3), distCoeff.at<double>(4));

        ROS_INFO( "Writing Camera Parameters" );
        cv::FileStorage fs640("Freiburg_ROS_default_640x480.xml", cv::FileStorage::WRITE);
        fs640 << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeff;
        fs640.release();

        cv::FileStorage fs320("Freiburg_ROS_default_320x240.xml", cv::FileStorage::WRITE);
        cv::Mat camMat320 = 0.5 * cameraMatrix;
        camMat320.at<double>(2,2) = 1.0f;
        fs320 << "cameraMatrix" << camMat320 << "distCoeffs" << distCoeff;
        fs320.release();
    }
    else
        ROS_ERROR( "Camera Params file not written...!");
#endif


}

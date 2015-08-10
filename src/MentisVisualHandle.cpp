///////////////////////////////////////////////////////////////////////////
//                        MentisVisualHandle                             //
///////////////////////////////////////////////////////////////////////////
//                                                                       //
//  Author : Manohar Kuse <mpkuse@ust.hk>                                //
//  Created on : 22nd July, 2015                                         //
//  Purpose : Defines a class for visualization to RViz.                 //
///////////////////////////////////////////////////////////////////////////


#include <MentisVisualHandle.h>
#include <SolveDVO.h>



MentisVisualHandle::MentisVisualHandle()
{
    isNodeHandleValid = false;
    rviz_frame_id = "denseVO"; //a default frameID

    poseAry.reserve(2000);
    poseAry.clear();
}

void MentisVisualHandle::setNodeHandle(SolveDVO *const dvoH )
{
    //
    /// Setup node handle <br/>
    this->dvoHandle = dvoH;
    isNodeHandleValid = true;


    //
    /// Setup publishers <br/>
    pub_inc_sp = dvoHandle->nh.advertise<visualization_msgs::Marker>( "/testVis/sphere", 100 );

    pub_pc = dvoHandle->nh.advertise<sensor_msgs::PointCloud>( "dvo/pointCloud", 1 );
    pub_global_pose = dvoHandle->nh.advertise<geometry_msgs::PoseStamped>( "dvo/globalPose", 1 );
    pub_path = dvoHandle->nh.advertise<nav_msgs::Path>("/dvo/path", 1 );

    //debug publishers
    pub_debug_keyFrames = dvoHandle->nh.advertise<visualization_msgs::MarkerArray>("/dvo/debug/keyFrame", 1 );


}


/// Set frameID. This is not a compulsary call as the default constructor sets a default string (denseVO)
void MentisVisualHandle::setRVizFrameID(const char *frameID)
{
    this->rviz_frame_id = frameID;
}

void MentisVisualHandle::incrementalSphere()
{

    ROS_INFO( "Publish a sphere........frameID : %s", rviz_frame_id );

    visualization_msgs::Marker marker;
    marker.header.frame_id = rviz_frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;


    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;

    marker.scale.x = 21.0;
    marker.scale.y = 21.0;
    marker.scale.z = 12.0;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    pub_inc_sp.publish(marker);
}



void MentisVisualHandle::publishCurrentPointCloud( int level )
{
    assert( dvoHandle->isCameraIntrinsicsAvailable && dvoHandle->isNowFrameAvailable );
    assert( dvoHandle->im_n.size() > 1 );


    sensor_msgs::PointCloud pcl_msg;
    pcl_msg.header.frame_id = rviz_frame_id;
    pcl_msg.header.stamp = ros::Time::now();


    float scaleFac = (float)pow(2,-level);
    Eigen::MatrixXf& _im = dvoHandle->im_n[level];
    Eigen::MatrixXf& _dim = dvoHandle->dim_n[level];

    sensor_msgs::ChannelFloat32 shade;
    shade.name = "intensity";


    for( int yy=0 ; yy<_im.rows() ; yy++ )
    {
        for( int xx=0 ; xx<_im.cols() ; xx++ )
        {
            float Z = _dim(yy,xx);
            if( Z < 10 )
                continue;
            float X = Z * (xx-scaleFac*dvoHandle->cx) / (scaleFac*dvoHandle->fx);
            float Y = Z * (yy-scaleFac*dvoHandle->cy) / (scaleFac*dvoHandle->fy);

            geometry_msgs::Point32 pt;
            pt.x = X;
            pt.y = Y;
            pt.z = Z;

            ROS_INFO_ONCE( "%.4f %.4f %.4f", X, Y, Z );

            pcl_msg.points.push_back(pt);
            shade.values.push_back( _im(yy,xx) );
        }
    }

    pcl_msg.channels.push_back(shade);
    pub_pc.publish( pcl_msg );
}




/// @brief Publishes the pose of kth frame with respect to global frame (ie. 1st frame)
/// Publishes to var `pub_final_pose`
void MentisVisualHandle::publishPoseFinal(Eigen::Matrix3f &rot, Eigen::Vector3f &tran)
{
    geometry_msgs::Pose rospose;
    matrixToPose(rot, tran, rospose);

    float yaw, pitch, roll;
    Eigen::Vector3f euler = rot.eulerAngles(2, 1, 0);
    yaw = euler(0,0);
    pitch = euler(1,0);
    roll = euler(2,0);


    ROS_INFO( "Publishing Pose : [ %.4f %.4f %.4f %.4f :: %.4f %.4f %.4f", rospose.orientation.x, rospose.orientation.y, rospose.orientation.z, rospose.orientation.w,
                     rospose.position.x, rospose.position.y, rospose.position.z );
    ROS_INFO( "Roll-Pitch-Yaw :  [ %f %f %f ] (camera POV)", roll, pitch, yaw );


    geometry_msgs::PoseStamped poseS;
    poseS.header.frame_id = rviz_frame_id;
    poseS.header.stamp = ros::Time::now();
    poseS.pose = rospose;
    poseAry.push_back(poseS);

    pub_global_pose.publish( poseS );

}

void MentisVisualHandle::publishPath()
{
    nav_msgs::Path pathMsg;
    pathMsg.header.frame_id = rviz_frame_id;
    pathMsg.header.stamp = ros::Time::now();
    for( int i=0 ; i<poseAry.size() ; i++ )
        pathMsg.poses.push_back(poseAry[i]);
    pub_path.publish(pathMsg);
}

void MentisVisualHandle::debug(Eigen::Matrix3f cR, Eigen::Vector3f cT)
{
    //
    // make 3d points of the reference frame as a fat matrix X.
    int level = 0;
    float scaleFac = (float)pow(2,-level);
    Eigen::MatrixXf _ref = dvoHandle->im_r[level];
    Eigen::MatrixXf _ref_depth = dvoHandle->dim_r[level];

    Eigen::MatrixXf XX = Eigen::MatrixXf::Constant( 3, _ref.rows()*_ref.cols(), 1.0f );
    Eigen::MatrixXf XX_int = Eigen::MatrixXf::Zero( 1, _ref.rows()*_ref.cols() ); //corresponding intensities


    int nC = 0;
    for( int yy=0 ; yy<_ref.rows() ; yy++ )
    {
        for( int xx=0 ; xx<_ref.cols() ; xx++ )
        {
            float Z = _ref_depth(yy,xx);
            float intensity = _ref(yy,xx);
            if( Z < 10.0f )
                continue;

            float X = Z * (xx-scaleFac*dvoHandle->cx) / (scaleFac*dvoHandle->fx);
            float Y = Z * (yy-scaleFac*dvoHandle->cy) / (scaleFac*dvoHandle->fy);

            XX(0,nC) = X;
            XX(1,nC) = Y;
            XX(2,nC) = Z;
            XX_int(0,nC) = intensity;
            nC++;
        }
    }

    ROS_INFO( "collected %d 3d points", nC );

    //
    // transform X~ = R.t() ( X - cT )
    dvoHandle->printRT( cR, cT, "MVIZ DEBUG");
    Eigen::MatrixXf cTRep;
    Eigen::Vector3f tmp_bogus(cT(0), cT(1), 5.0f);
    igl::repmat(tmp_bogus,1,XX.cols(),cTRep); // repeat cT col-wise (uses IGL library)
    Eigen::MatrixXf XX_transformed = cR.transpose() * ( XX - cTRep ); //this would be X~


    ROS_INFO_STREAM( "rep :\n"<< cTRep.topLeftCorner(3, 7) );
    ROS_INFO_STREAM( "XX :\n"<< XX.topLeftCorner(3, 7) );



    //
    // project X~ on the image
    Eigen::Matrix3f scaleMatrix = Eigen::Matrix3f::Identity();
    scaleMatrix(0,0) = scaleFac;
    scaleMatrix(1,1) = scaleFac;

    Eigen::ArrayXXf lastRow_inv = XX_transformed.row(2).array().inverse();
    for( int i=0 ; i<3 ; i++ )
        XX_transformed.row(i).array() *= lastRow_inv;


    Eigen::MatrixXf _2d_reprojected = scaleMatrix * dvoHandle->K * XX_transformed;


    //
    // convert to image (2d matrix) NxN

    Eigen::MatrixXf image = Eigen::MatrixXf::Zero(_ref.rows(), _ref.cols());
    for( int i=0 ; i<nC ; i++ )
    {
        int xx = (int)_2d_reprojected(0,i);
        int yy = (int)_2d_reprojected(1,i);


        int inten = XX_int(0,i);

        if( yy>0 && yy<image.rows()-1 && xx>0 && xx<image.cols()-1 )
        {
            image(yy,xx) = inten;
//            image(yy+1,xx) = inten;
//            image(yy,xx+1) = inten;
//            image(yy+1,xx+1) = inten;
        }
    }


    dvoHandle->imshowEigenImage( "debug full reprojection", image );

}

void MentisVisualHandle::publishGOP()
{
    nav_msgs::Path pathMsg;
    pathMsg.header.frame_id = rviz_frame_id;
    pathMsg.header.stamp = ros::Time::now();

    geometry_msgs::PoseStamped st;
    for( int i=0 ; i<dvoHandle->gop.size() ; i++ )
    {
        st.header = pathMsg.header;
        st.pose = dvoHandle->gop.getGlobalPoseAt(i);
        pathMsg.poses.push_back( st );
    }


#ifdef __DEBUG_FRAME_MARKER__
    //marking keyframes
    visualization_msgs::MarkerArray markerAry;
    for( int i=0 ; i<dvoHandle->gop.size() ; i++ )
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = rviz_frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";

        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;


        marker.pose = dvoHandle->gop.getGlobalPoseAt(i);

        if( dvoHandle->gop.isKeyFrameAt(i) )
        {
            marker.id = i;
            marker.scale.x = .03;
            marker.scale.y = .03;
            marker.scale.z = .03;

            int reason = dvoHandle->gop.getReasonAt(i);
            switch(reason)
            {
            case 1: //init
                marker.color.r = 1.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0f;
                break;
            case 2: //laplacian thresh breached
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0f;
                break;
            case 3: //ratio of visible less than `thresh`
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0f;
                break;
            case 4: //# of reprojected pts less than 50
                marker.color.r = 1.0f;
                marker.color.g = 1.0f;
                marker.color.b = 1.0f;
                marker.color.a = 1.0f;
                break;

            }
            markerAry.markers.push_back(marker);
        }
#ifdef __DEBUG_FRAME_MARKER_ALL_FRAMES
        else //non-key frames
        {
            marker.id = i;
            marker.scale.x = .01;
            marker.scale.y = .01;
            marker.scale.z = .01;
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
            markerAry.markers.push_back(marker);
        }
#endif
    }
    pub_debug_keyFrames.publish(markerAry);
#endif //__DEBUG_FRAME_MARKER__


    pub_path.publish(pathMsg);
    pub_global_pose.publish(st);

}



/// @brief Given the rotation and translation matrix convert to ros Pose representation
/// @param[in] rot : 3x3 rotation matrix
/// @param[in] trans : 3-vector representing translation
/// @param[out] rosPose : geometry_msgs::Pose as output
void MentisVisualHandle::matrixToPose(Eigen::Matrix3f rot, Eigen::Vector3f tran, geometry_msgs::Pose& rospose)
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


#include <SolvePnP.h>


/// default constructor. Init the subscribers
SolvePnP::SolvePnP()
{
    char camPramFileName[] = "params.xml";
    setCameraMatrix( camPramFileName );


    sub = nh.subscribe( "odometry/rgbd", 1, &SolvePnP::imageArrivedCallBack, this );

    pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/pnp/points_3d_array",10);
    pub_pose = nh.advertise<geometry_msgs::PoseArray>("/pnp/pose_array", 10 );
    pub_pose_1 = nh.advertise<geometry_msgs::PoseStamped>("/pnp/pose", 10 );
    pub_path = nh.advertise<nav_msgs::Path>("pnp/path", 10 );


}



/// Input opencv XML file containing camera internal params and distortion coif.
/// Note: At this moment the distortion coif. are not used.
void SolvePnP::setCameraMatrix(char* calibFile)
{
    //
    // Read calibration matrix, distortion coiffs
    //
    cv::FileStorage fs(calibFile, cv::FileStorage::READ);
    if( fs.isOpened() == false )
    {
        ROS_ERROR_STREAM( "[SolvePnP::setCameraMatrix] Error opening camera "
                "params file : "<< calibFile );
        return;
    }

    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    //cout<< "Camera Matrix : \n"<<  cameraMatrix << endl;
    //cout<< "Distortion Coifs : \n"<< distCoeffs << endl;
    fx = cameraMatrix.at<double>(0,0);
    fy = cameraMatrix.at<double>(1,1);
    cx = cameraMatrix.at<double>(0,2);
    cy = cameraMatrix.at<double>(1,2);

    K = Eigen::Matrix3d::Zero();
    K(0,0) = fx;
    K(1,1) = fy;
    K(0,2) = cx;
    K(1,2) = cy;
    K(2,2) = 1.0;

    ROS_INFO( "[SolvePnP::setCameraMatrix] Camera Matrix & Distortion Coif Loaded");
    ROS_INFO( "fx=%.4f, fy=%.4f, cx=%.4f, cy=%.4f", fx, fy, cx, cy );


    cameraIntrinsicsReady = true;

}




/// @brief Callback to receive RGBD from the topic "odometry/rgbd"
///
/// The subscriber is defined in the construction. This function is the callback for the same.
/// This is a blocking function. Thus do not do any processing here.
void SolvePnP::imageArrivedCallBack( rgbd_odometry::RGBDFrameConstPtr msg )
{
    ROS_INFO_STREAM_ONCE( "1st RGBD frame received. Will continue receiving but not report anymore on this");
    isFrameAvailable=false;


    cv::Mat frame, dframe;
    try
    {
        frame =  cv_bridge::toCvShare(  msg->frame, msg, "bgr8" )->image ;
        dframe =  cv_bridge::toCvShare(  msg->dframe, msg, "mono16" )->image ;
    }
    catch( cv_bridge::Exception& e )
    {
        ROS_ERROR( "cv_bridge exception: %s", e.what() );
        isFrameAvailable = false;
        return;
    }

    dframe.setTo(1, (dframe==0) ); //to avoid zero depth

    frame.copyTo( this->rcvd_frame );
    dframe.copyTo( this->rcvd_dframe );
    isFrameAvailable=true;

}

bool SolvePnP::getChessBoardPts(Eigen::MatrixXd &objPts, Eigen::MatrixXd &imPts)
{
    //using the current image
    assert( this->isFrameAvailable && "[SolvePnP::getChessBoardPts] frame not available" );

    std::vector<cv::Point2f> corners;
    ROS_INFO( "start check for chessboard");
    bool status = cv::findChessboardCorners(this->rcvd_frame, cv::Size(9,6), corners, cv::CALIB_CB_FAST_CHECK);
    if( status == false )
        return false;


    ROS_INFO( "FOUND %d Chessboard corners", corners.size());
    imPts = Eigen::MatrixXd((int)corners.size(),2);
    objPts = Eigen::MatrixXd((int)corners.size(),3);
    cv::Mat& im = this->rcvd_frame;
    char text[50];

    int co=0;
    for( int i=(corners.size()-1); i>=0 ; i--, co++ )
    {

        imPts(i,0) = corners[i].x;
        imPts(i,1) = corners[i].y;


        objPts(i,0) = /*.025**/(co%9);
        objPts(i,1) = /*.025**/floor(co/9);
        objPts(i,2) = 0.0;

        int rowN = corners[i].y;
        int colN = corners[i].x;
        im.at<cv::Vec3b>(rowN,colN)[0] = 0;
        im.at<cv::Vec3b>(rowN,colN)[1] = 0;
        im.at<cv::Vec3b>(rowN,colN)[2] = 255;


        //snprintf( text, 40, "%2.2f", objPts(i,0));
        //cv::putText( im, text, corners[i], cv::FONT_HERSHEY_COMPLEX_SMALL, .5, cv::Scalar(0,0,255) );
        //if( i==53 || i==52)
        //    cv::circle(im, corners[i], 10, cv::Scalar(0,0,255), 3 );
    }
    return true;
}


/// @brief Solves the 3d-2d PnP problem with Gauss-Newton Iterations
/// @param[in] objPts : Nx3 matrix with co-ordinates of N points
/// @param[in] imPts  : Nx2 matrix with imaged pts. 1st col represents x (ie. colIndx), 2nd col represents y (ie. rowIndx)
/// @param[in,out] R : input the initial rotation matrix. Will overwrite it with final rotation
/// @param[in,out] T : input the initial translation vector. Will overwrite it with final translation
void SolvePnP::PnP(Eigen::MatrixXd &objPts, Eigen::MatrixXd &imPts, Eigen::Matrix3d &R, Eigen::Vector3d &T)
{
    assert( cameraIntrinsicsReady );
    assert(objPts.rows() > 0 && objPts.cols() == 3 );
    assert(imPts.rows() > 0 && imPts.cols() == 2 );


    ROS_INFO( "Start of Gauss Newton Iterations");
    for( int i=0 ; i<5 ; i++ ) //gauss-newton iterations
    {
        ROS_DEBUG( "------ Iteration #%d ------", i );
        //store R, T for visualization
        stored_R.push_back(R);
        stored_T.push_back(T);
        ROS_DEBUG_STREAM("R :\n"<< R );
        ROS_DEBUG_STREAM("T :\n"<< T );

        Eigen::MatrixXd J_i; //jacobian of 1 correspondence (2x6 matrix)
        Eigen::Vector2d r_i; //residue of 1 correspondence (2x1)

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6,6);
        Eigen::VectorXd e = Eigen::VectorXd::Zero(6);
        for( int i=0 ; i<objPts.rows() ; i++ ) //for each correspondence
        {
            computeJacobian( i, objPts, imPts, R, T, J_i );
            computeResidue(  i, objPts, imPts, R, T, r_i );

            H += J_i.transpose() * J_i;
            e += J_i.transpose() * r_i;
        }

        H = -H;

        // solve :  H * del = e
        Eigen::VectorXd del = H.colPivHouseholderQr().solve(e);
        //ROS_INFO_STREAM( "del :\n"<< del );

        Eigen::Matrix3d outR;
        Eigen::Vector3d outT;
        exponentialMap(del,outR, outT);

        T = R*outT + T;
        R = R * outR;



    }

    // final R, T
    stored_R.push_back(R);
    stored_T.push_back(T);
    ROS_INFO( "End of Gauss Newton Iterations");


}

void SolvePnP::opencvPnP(Eigen::MatrixXd &objPts, Eigen::MatrixXd &imPts, Eigen::Matrix3d &R, Eigen::Vector3d &T)
{
    assert( cameraIntrinsicsReady );
    assert( objPts.rows()>0 && objPts.cols() == 3 );
    assert( imPts.rows()>0 && imPts.cols() == 2 );

    std::vector<cv::Point3f> oPts; //object points in opencv format
    std::vector<cv::Point2f> iPts; //image pts

    for( int i=0 ; i<objPts.rows() ; i++ )
    {
        cv::Point3f otmp( objPts(i,0), objPts(i,1), objPts(i,2) );
        cv::Point2f itmp( imPts(i,0), imPts(i,1) );

        oPts.push_back(otmp);
        iPts.push_back(itmp);
    }



    cv::Mat cvR, cvRVec;
    cv::eigen2cv(R,cvR);
    cv::Rodrigues(cvR, cvRVec );

    cv::Mat cvT;
    cv::eigen2cv(R, cvT );

    cv::solvePnP(oPts, iPts, cameraMatrix, cv::Mat(), cvRVec, cvT, false );

    cv::Rodrigues(cvRVec, cvR );
    cv::cv2eigen( cvR, R );


    cv::cv2eigen( cvT, T );


    ROS_INFO_STREAM( "R:::\n"<< R );
    ROS_INFO_STREAM( "T:::\n"<< T );

    stored_R.push_back(R);
    stored_T.push_back(T);

}

/// @brief Given the obj<-->img pt pair computes the Jacobian (2x6)
/// @param [in] index : Index of the correspondence
/// @param[in] objPts : Nx3 matrix with co-ordinates of N points
/// @param[in] imPts  : Nx2 matrix with imaged pts. 1st col represents x (ie. colIndx), 2nd col represents y (ie. rowIndx)
void SolvePnP::computeJacobian(int index, Eigen::MatrixXd &objPts, Eigen::MatrixXd &imPts, Eigen::Matrix3d &R, Eigen::Vector3d &T, Eigen::MatrixXd &J_i)
{
    // Create Pw, Pb
    double x = objPts(index,0);
    double y = objPts(index,1);
    double z = objPts(index,2);
    Eigen::Vector3d Pw(x,y,z); //point P wrt world origin
    Eigen::Vector3d Pb = R.transpose() * (Pw - T); //point P wrt body (ie. camera) origin

    // Image co-ordinates
    double u = imPts(index,0);
    double v = imPts(index,1);


    //
    // J = -1 * A1 * A2
    //

    // Compute A1 (2x3)
    Eigen::MatrixXd A1(2,3);
    A1(0,0) = 1/Pb(2);
    A1(0,1) = 0;
    A1(0,2) = -Pb(0)/(Pb(2)*Pb(2));
    A1(1,0) = 0;
    A1(1,1) = 1/Pb(2);
    A1(1,2) = -Pb(1)/(Pb(2)*Pb(2));


    // Compute A2 (3x6)
    Eigen::MatrixXd A2(3,6);
    A2 = Eigen::MatrixXd::Zero(3,6);

    Eigen::Vector3d tmp = R.transpose() * (Pw - T);
    Eigen::Matrix3d tmpx;
    to_se_3(tmp, tmpx);

    A2.topLeftCorner(3,3) = -R.transpose();
    A2.topRightCorner(3,3) = tmpx;

    // J
    J_i = -1.0 * A1 * A2;

}



void SolvePnP::computeResidue(int index, Eigen::MatrixXd &objPts, Eigen::MatrixXd &imPts, Eigen::Matrix3d &R, Eigen::Vector3d &T, Eigen::Vector2d &r_i)
{
    // Create Pw, Pb
    double x = objPts(index,0);
    double y = objPts(index,1);
    double z = objPts(index,2);
    Eigen::Vector3d Pw(x,y,z); //point P wrt world origin
    Eigen::Vector3d Pb = R.transpose() * (Pw - T); //point P wrt body (ie. camera) origin


    // Image co-ordinates
    double u = imPts(index,0);
    double v = imPts(index,1);

    Eigen::Vector3d U_0(u,v,1.0);
    Eigen::Vector3d U_n = K.inverse() * U_0;
    Eigen::Vector2d U_hat( U_n(0)/U_n(2), U_n(1)/U_n(2) );


    Eigen::Vector2d U;
    U(0) = Pb(0)/Pb(2);
    U(1) = Pb(1)/Pb(2);

    r_i = U_hat - U;

}


/// \brief Given a 6-DOF psi, does an exponential Map.
///
/// @param[in] w: Psi 6-vector
/// @param[out] : output transformation matrix
void SolvePnP::exponentialMap(Eigen::VectorXd& psi, Eigen::Matrix3d& outR, Eigen::Vector3d& outT) {
    assert(psi.rows() == 6 && "PSI does not seem to have 6 rows");

    Eigen::Vector3d t = psi.head(3);
    Eigen::Vector3d w = psi.tail(3);



    Eigen::Matrix3d wx;
    to_se_3(w,wx);


    double theta = w.norm();
    ROS_DEBUG( "THETA = %lf", theta );
    if( theta < 1E-12 )
    {
        //outTr = Eigen::Matrix4d::Identity();
        outR = Eigen::Matrix3d::Identity();
        outT = Eigen::Vector3d::Zero();
        return;
    }

    Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
    Eigen::RowVector3d zer_row = Eigen::RowVector3d::Zero();


    Eigen::Matrix3d xR = I3 + sin(theta)/theta * wx + (1.0-cos(theta))/(theta*theta) * wx * wx;

    Eigen::Matrix3d xV = I3 + (1-cos(theta))/(theta*theta) * wx + (theta-sin(theta))/(theta*theta*theta) * wx *wx;


    outR = xR;
    outT = xV*t;
    //outTr << xR, xV*t, zer_row, 1.0;
}


/// @brief Given 3d points, R, T, K. Computes the projection of these points
/// @param[in] objectPts : 3d coordinates of object points. 1st col is X, 2nd Y, 3rd Z. Expressed in world co-ordinate system
/// @param[in] R : Rotation matrix representing the body frame expressed in world frame.
/// @param[in] T : Translation vector representing the body frame expressed in world frame.
/// @param[out] imReproj : The image canvas to use for reprojection
void SolvePnP::getReprojectedImage(Eigen::MatrixXd &objectPts, Eigen::Matrix3d R, Eigen::Vector3d T, cv::Mat &imReproj)
{
    for( int i=0 ; i<objectPts.rows() ; i++ )
    {
        double x = objectPts(i,0);
        double y = objectPts(i,1);
        double z = objectPts(i,2);
        Eigen::Vector3d Pw(x,y,z); //point P wrt world origin
        Eigen::Vector3d Pb = R.transpose() * (Pw - T); //point P wrt body (ie. camera) origin

        Eigen::Vector3d U = K * Pb;
        Eigen::Vector2d U_hat( U(0)/U(2), U(1)/U(2) );

        if( i==0 || i==1 )
            cv::circle( imReproj, cv::Point2f(U_hat(0), U_hat(1)), 5, cv::Scalar(255,0,255), 1 );
        else
            cv::circle( imReproj, cv::Point2f(U_hat(0), U_hat(1)), 5, cv::Scalar(0,0,255), 1 );
    }
}



/// \brief Converts a vector to its se(3) matrix
///
/// @param[in] w : a 3 vector. May not be unit vector
/// @param[out] wx : se(3) of w
void SolvePnP::to_se_3(Eigen::Vector3d& w, Eigen::Matrix3d& wx) {
    wx = Eigen::Matrix3d::Zero();

    wx(1,2) = -w(0);
    wx(0,2) =  w(1);
    wx(0,1) = -w(2);

    wx(2,1) =  w(0);
    wx(2,0) = -w(1);
    wx(1,0) =  w(2);
}



void SolvePnP::publish3dPoints(Eigen::MatrixXd objectPts)
{
    ROS_INFO_STREAM_THROTTLE( 30, "publish sphere, rows:"<<objectPts.rows() );

    visualization_msgs::MarkerArray array;
    array.markers.resize(objectPts.rows());
    for( int i=0 ; i<objectPts.rows() ; i++ )
    {
        visualization_msgs::Marker msg;
        msg.header.frame_id = "odom";
        msg.header.stamp = ros::Time::now();
        msg.id = i;

        msg.type = visualization_msgs::Marker::SPHERE;
        msg.action = visualization_msgs::Marker::ADD;

        msg.pose.position.x = objectPts(i,0);
        msg.pose.position.y = objectPts(i,1);
        msg.pose.position.z = objectPts(i,2);
        msg.scale.x = msg.scale.y = msg.scale.z = .3;
        if( i==0)
        {
            msg.color.r = 1.0;
            msg.color.g = 0.0;
            msg.color.b = 1.0;
            msg.color.a = 1.0;
        }
        else if( i==1)
        {
            msg.color.r = 1.0;
            msg.color.g = 0.0;
            msg.color.b = 1.0;
            msg.color.a = 1.0;
        }
        else
        {
            msg.color.r = 1.0;
            msg.color.g = 0.0;
            msg.color.b = 0.0;
            msg.color.a = 1.0;
        }


        array.markers.push_back(msg);


    }

    pub_markers.publish(array);


}

void SolvePnP::publishPoses(bool publishFinalPose=true, bool publishIntermediatePoses=false )
{
    ROS_INFO_THROTTLE( 30, "===========================\npublish %d poses", stored_R.size() );

    geometry_msgs::PoseArray ary;
    ary.header.frame_id = "odom";
    ary.header.stamp = ros::Time::now();


    if( publishFinalPose )
    {
        // publishing only the last pose
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header =  ary.header;
        Eigen::Matrix3d finalR = stored_R.back();
        Eigen::Vector3d finalT = stored_T.back();
        Eigen::Quaterniond quat(finalR);
        pose_stamped.pose.position.x = finalT(0);
        pose_stamped.pose.position.y = finalT(1);
        pose_stamped.pose.position.z = finalT(2);
        pose_stamped.pose.orientation.x = quat.x();
        pose_stamped.pose.orientation.y = quat.y();
        pose_stamped.pose.orientation.z = quat.z();
        pose_stamped.pose.orientation.w = quat.w();
        pub_pose_1.publish(pose_stamped);

        prevPoses.push_back(pose_stamped.pose);

        nav_msgs::Path path;
        path.header = pose_stamped.header;
        for( int i=0 ; i<prevPoses.size() ; i++ )
        {
            geometry_msgs::PoseStamped ps2;
            ps2.header = ary.header;
            ps2.pose = prevPoses[i];
            path.poses.push_back(ps2);
        }
        pub_path.publish( path );

    }



    if( publishIntermediatePoses )
    {
        for( int i=0 ; i<stored_R.size() ; i++ )
        {
            geometry_msgs::Pose pose;
            Eigen::Matrix3d sR = stored_R[i];
            Eigen::Vector3d sT = stored_T[i];


            ROS_INFO_STREAM_THROTTLE(30, "R :\n"<< sR );
            ROS_INFO_STREAM_THROTTLE(30, "T :\n"<< sT );

            Eigen::Quaterniond quat(sR);
            pose.position.x = sT(0);
            pose.position.y = sT(1);
            pose.position.z = sT(2);
            pose.orientation.x = quat.x();
            pose.orientation.y = quat.y();
            pose.orientation.z = quat.z();
            pose.orientation.w = quat.w();

            ary.poses.push_back(pose);

        }

        pub_pose.publish(ary);
    }


}



/// @brief The event loop. Basically an ever running while with ros::spinOnce()
/// This is a re-implementation taking into care the memory scopes and also processing only points with higher image gradients
void SolvePnP::eventLoop()
{
    // Initialize a solution
    Eigen::Matrix3d iR;
    Eigen::Vector3d iT;
//    iR = Eigen::AngleAxisd(-0.3, Eigen::Vector3d(1.0, 0.0, 0.0));
//    iT = Eigen::Vector3d(0.0, 2.0, -25.);

    iR = Eigen::AngleAxisd(0., Eigen::Vector3d(1.0, 0.0, 0.0));
    iT = Eigen::Vector3d(0.0, 15.0, -25.);

    ROS_INFO_STREAM( "matrix : \n"<< iR );
    ROS_INFO_STREAM( "trans : \n"<< iT );

    Eigen::MatrixXd objectPts; // N x 3
    Eigen::MatrixXd imPts;     // N x 2




    ros::Rate rate(30);
    while( ros::ok() )
    {
        ros::spinOnce();
        if( !(this->isFrameAvailable) )
            continue;


        bool status = getChessBoardPts( objectPts, imPts );

        this->stored_R.clear();
        this->stored_T.clear();
        if( status )
        {
            PnP( objectPts, imPts, iR, iT );
            publish3dPoints(objectPts);
            publishPoses();
        }
        else
            ROS_INFO( "No chessboard pattern");






        cv::imshow( "im", this->rcvd_frame );
        cv::waitKey(3);

        rate.sleep();
    }




    /*
    cv::Mat imtmp = cv::imread( "chess.png" );
    cv::Mat imReprojInit = cv::Mat::zeros(imtmp.rows,imtmp.cols,imtmp.type());
    cv::Mat imReprojFinal = cv::Mat::zeros(imtmp.rows,imtmp.cols,imtmp.type());

    imtmp.copyTo( this->rcvd_frame );
    isFrameAvailable=true;

    bool status = getChessBoardPts( objectPts, imPts );
//    cv::imshow("win", this->rcvd_frame);
//    cv::waitKey(0);
//    return 0;

    getReprojectedImage( objectPts, iR, iT, imReprojInit );

//    if( status )
//        opencvPnP( objectPts, imPts, iR, iT );

//    stored_R.clear();
//    stored_T.clear();

    if( status )
        PnP( objectPts, imPts, iR, iT );




    getReprojectedImage( objectPts, iR, iT, imReprojFinal );


    ROS_INFO( "Done...!" );

    ros::Rate rate(1);
    //for(int i=0 ; i<4 && ros::ok(); i++ )
    while( ros::ok() )
    {
        ros::spinOnce();
        publish3dPoints(objectPts);
        publishPoses();

        cv::imshow( "original", this->rcvd_frame );
        cv::imshow( "reprojected(initial)", imReprojInit );
        cv::imshow( "reprojected(final)", imReprojFinal );
        cv::waitKey(30);

        rate.sleep();
    }
    */


}



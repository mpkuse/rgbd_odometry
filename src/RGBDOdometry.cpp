#include <RGBDOdometry.h>
#include <EPoseEstimator.h>

#include <log4cxx/logger.h>


RGBDOdometry::RGBDOdometry()
{
    //setting logger level
    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
    ros::console::notifyLoggerLevelsChanged();


    sub = nh.subscribe( "odometry/rgbd", 5, &RGBDOdometry::imageArrivedCallBack, this );
    odom_pub = nh.advertise<nav_msgs::Odometry>("/topicodom", 5, true );
    path_pub = nh.advertise<nav_msgs::Path>("/topicpath", 5, true );
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/topicpose", 5, true );

    cameraIntrinsicsReady = false;
    isFrameAvailable = false;
    isRefFrameAvailable = false;
    isNowFrameAvailable = false;
    isPyramidalRefFrameAvailable = false;
    isPyramidalNowFrameAvailable = false;
    isJacobiansAvailable = false;

    char camPramFileName[] = "params.xml";
    setCameraMatrix( camPramFileName );


    // Const Parameters
    const_gradientThreshold = 5;
    const_maxJacobianSize = 50000;
    const_minimumRequiredPts = 100;


}


/// Input opencv XML file containing camera internal params and distortion coif.
/// Note: At this moment the distortion coif. are not used.
void RGBDOdometry::setCameraMatrix(char* calibFile)
{
    //
    // Read calibration matrix, distortion coiffs
    //
    cv::FileStorage fs(calibFile, cv::FileStorage::READ);
    if( fs.isOpened() == false )
    {
        ROS_ERROR_STREAM( "[RGBDOdometry::setCameraMatrix] Error opening camera "
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

    ROS_INFO( "Camera Matrix & Distortion Coif Loaded");
    ROS_INFO( "fx=%.4f, fy=%.4f, cx=%.4f, cy=%.4f", fx, fy, cx, cy );


    cameraIntrinsicsReady = true;

}

/*
/// @brief The event loop. Basically an ever running while with ros::spinOnce()
void RGBDOdometry::eventLoop()
{

    EPoseEstimator eps;
    eps.setCameraMatrix("params.xml");

    Eigen::Matrix3d initR = Eigen::Matrix3d::Identity();
    Eigen::Vector3d initT = Eigen::Vector3d::Zero();


    int nFrame = 0;
    while( ros::ok() )
    {
        ros::spinOnce();
        if( !(this->isFrameAvailable) )
            continue;

        if( (nFrame % 30) == 0)
        {
            setRefFrame( this->rcvd_frame, this->rcvd_dframe );
            eps.setRefFrame(this->im_r_color, this->dim_r);
        }

        setNowFrame( this->rcvd_frame, this->rcvd_dframe );
        eps.setNowFrame(this->im_color, this->dim);

        initR = Eigen::Matrix3d::Identity();
        initT = Eigen::Vector3d::Zero();

        eps.setPyramidalImages(2);
        eps.estimate( initR, initT );


        cv::imshow( "ref", this->im_r);
        cv::imshow( "now", this->im );
//        cv::imshow( "ref0", this->_im_r[0]);
//        cv::imshow( "ref1", this->_im_r[1]);
//        cv::imshow( "ref2", this->_im_r[2]);
//        cv::imshow( "ref3", this->_im_r[3]);
//        cv::imshow( "now0", this->_im[0]);
//        cv::imshow( "now1", this->_im[1]);
//        cv::imshow( "now2", this->_im[2]);
//        cv::imshow( "now3", this->_im[3]);
        cv::waitKey(30);

        nFrame++;

    }
}
*/


/// @brief The event loop. Basically an ever running while with ros::spinOnce()
/// This is a re-implementation taking into care the memory scopes and also processing only points with higher image gradients
void RGBDOdometry::eventLoop()
{
    int nFrame = 0;
    TransformRep T = Eigen::AngleAxisd(0.0, Eigen::Vector3d(1.,0.,0.)) * Eigen::Translation3d(0.,0.,0.) ;
    TransformRep base = Eigen::AngleAxisd(0.0, Eigen::Vector3d(1.,0.,0.)) * Eigen::Translation3d(0.,0.,0.) ;

    nav_msgs::Path path;
    nav_msgs::Odometry odom;
    geometry_msgs::PoseStamped s_rospose;


    while( ros::ok() )
    {
        ros::spinOnce();
        if( !(this->isFrameAvailable) )
            continue;

        // renew ref-frame every 30 frames
        if( (nFrame % 10000) == 0)
        {
            ROS_INFO( "--- New Reference Frame ---");
            base = base * T;
            setRefFrame( this->rcvd_frame, this->rcvd_dframe );
            T = Eigen::AngleAxisd(0.0, Eigen::Vector3d(1.,0.,0.)) * Eigen::Translation3d(0.,0.,0.) ;

            computeJacobianAllLevels();
        }

        ROS_INFO( "--- Processing Frame #%d ---", nFrame );
        setNowFrame( this->rcvd_frame, this->rcvd_dframe );

        // At this point the ref, jacobians, now frames are available. Run the gauss-newton iterations for evaluate pose of now-frame wrt to ref-frame

        //T = Eigen::AngleAxisd(0.0, Eigen::Vector3d(1.,0.,0.)) * Eigen::Translation3d(0.,0.,0.) ;
        gaussNewtonIterations(3, T);
        gaussNewtonIterations(2, T);
        //gaussNewtonIterations(1, T);


        //trial for publishing

        ros::Time current_time = ros::Time::now();
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        path.header = odom.header;
        s_rospose.header = odom.header;



        TransformRep toSend = base * T;
        Eigen::Vector3d xtrans =  toSend.translation();
        Eigen::Matrix3d xrot = toSend.rotation();
        Eigen::Quaterniond quat(xrot);

        geometry_msgs::Pose rospose;


        rospose.position.x = 1000*xtrans(0);
        rospose.position.y = 1000*xtrans(1);
        rospose.position.z = 1000*xtrans(2);
        rospose.orientation.x = quat.x();
        rospose.orientation.y = quat.y();
        rospose.orientation.z = quat.z();
        rospose.orientation.w = quat.w();

        odom.pose.pose = rospose;
        s_rospose.pose = rospose;
        path.poses.push_back(s_rospose);



        odom_pub.publish(odom);
        path_pub.publish(path);
        pose_pub.publish(s_rospose);

        // end of publishing code

        //showFrames();
        //cv::waitKey(30);
        nFrame++;
        this->isFrameAvailable = false;

    }
}





/// @brief Callback to receive RGBD from the topic "odometry/rgbd"
///
/// The subscriber is defined in the construction. This function is the callback for the same.
/// This is a blocking function. Thus do not do any processing here.
void RGBDOdometry::imageArrivedCallBack( rgbd_odometry::RGBDFrameConstPtr msg )
{
    //ROS_INFO_STREAM( "msg::: Name:"<< msg->name << " Age:"<< msg->age );

    ROS_INFO_STREAM_ONCE( "1st RGBD frame received. Will continue receiving but not report anymore on this");

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

//    ROS_INFO_STREAM( dframe.at<uint16_t>(200,200) );
//    cv::imshow("viewz", frame );
//    cv::imshow("viewz_depth", dframe );
    //    cv::waitKey(30);

}

/// @brief Helper function to display images
void RGBDOdometry::showFrames()
{
//    cv::imshow( "ref", this->im_r);
//    cv::imshow( "now", this->im );
        cv::imshow( "ref0", this->_im_r[0]);
        cv::imshow( "ref1", this->_im_r[1]);
        cv::imshow( "ref2", this->_im_r[2]);
        cv::imshow( "ref3", this->_im_r[3]);
        cv::imshow( "now0", this->_im[0]);
        cv::imshow( "now1", this->_im[1]);
        cv::imshow( "now2", this->_im[2]);
        cv::imshow( "now3", this->_im[3]);
        cv::waitKey(30);
}

void RGBDOdometry::sOverlay(cv::Mat im, Eigen::MatrixXi mask)
{

    assert( im.data != NULL );
    assert( (im.rows == mask.rows()) && "Image and mask rows must match");
    assert( (im.cols == mask.cols()) && "Image and mask cols must match");
    assert( (im.channels() == 3) && "im must be 3 channel");

    cv::Mat xim = im.clone();

    for( int j=0 ; j<mask.cols() ; j++ )
    {
        for( int i=0 ; i<mask.rows() ; i++ )
        {
            if( mask(i,j) > 0 )
            {
                xim.at<cv::Vec3b>(i,j)[0] = 0;
                xim.at<cv::Vec3b>(i,j)[1] = 0;
                xim.at<cv::Vec3b>(i,j)[2] = 255;
            }
        }
    }

    cv::imshow( "marked_image", xim );
    cv::waitKey(30);
}

void RGBDOdometry::setRefFrame(cv::Mat rgb, cv::Mat depth)
{
    assert( isFrameAvailable && "Frame not retrived to set in refFrame");
    isPyramidalRefFrameAvailable = false;
    rgb.copyTo(this->im_r_color);
    cv::cvtColor(rgb, this->im_r, CV_BGR2GRAY);
    depth.copyTo(this->dim_r);

    isRefFrameAvailable = true;


    float scaleFactor = 1.0;
    cv::Mat tmp, tmpdep, tmpcolor;


    this->_im_r.clear();
    this->_dim_r.clear();
    this->_im_r_color.clear();
    for( int i=0 ; i<4 ; i++ )
    {
        cv::resize(this->im_r, tmp, cv::Size(), scaleFactor, scaleFactor, cv::INTER_NEAREST ); //for gray image
        cv::resize(this->dim_r, tmpdep, cv::Size(), scaleFactor, scaleFactor, cv::INTER_NEAREST ); //for depth image
        cv::resize(this->im_r_color, tmpcolor, cv::Size(), scaleFactor, scaleFactor, cv::INTER_NEAREST ); //for color image

        this->_im_r.push_back(tmp);
        this->_dim_r.push_back(tmpdep);
        this->_im_r_color.push_back(tmpcolor);
        scaleFactor /= 2.;
    }
    isPyramidalRefFrameAvailable = true;

}

void RGBDOdometry::setNowFrame(cv::Mat rgb, cv::Mat depth)
{
    assert( isFrameAvailable && "Frame not retrived to set in nowFrame");
    isPyramidalNowFrameAvailable = false;
    rgb.copyTo(this->im_color);
    cv::cvtColor(rgb, this->im, CV_BGR2GRAY);
    depth.copyTo(this->dim);
    isNowFrameAvailable = true;


    float scaleFactor = 1.0;
    cv::Mat tmp, tmpdepth, tmpcolor;


    this->_im.clear();
    this->_dim.clear();
    this->_im_color.clear();
    for( int i=0 ; i<4 ; i++ )
    {
        cv::resize(this->im, tmp, cv::Size(), scaleFactor, scaleFactor, cv::INTER_NEAREST );
        cv::resize(this->dim, tmpdepth, cv::Size(), scaleFactor, scaleFactor, cv::INTER_NEAREST );
        cv::resize(this->im_color, tmpcolor, cv::Size(), scaleFactor, scaleFactor, cv::INTER_NEAREST );
        this->_im.push_back(tmp);
        this->_dim.push_back(tmpdepth);
        this->_im_color.push_back(tmpcolor);
        scaleFactor /= 2.;
    }
    isPyramidalNowFrameAvailable = true;
}


/// @brief For each of the pyramid level computes the jacobian matrix.size X x 6.
/// @note The Jacobians and A (J'*J) are stored in std::vector of the corresponding type, with a deep copy.
/// @note vector[0] is not to be used, it stands for the 0th level ie. the base. It is not computed since itz not real time
void RGBDOdometry::computeJacobianAllLevels()
{
    isJacobiansAvailable = false;

    _J.clear();
    _A.clear();
    _roimask.clear();
    Eigen::MatrixXd jacobian,A; // A:= J'*J
    Eigen::MatrixXi roimask; //=1 for points with higher im-gradient, zero else

    _J.push_back( Eigen::MatrixXd::Zero(5,5)); //place-holder, since we do not compute jacobian for level-0 of the pyramid. Reason:computationally expensive
    _A.push_back( Eigen::MatrixXd::Zero(5,5));
    _roimask.push_back( Eigen::MatrixXi::Zero(5,5));
    for( int i=1; i<=3 ; i++ ) //loop over levels
    {
        computeJacobian(i, jacobian, roimask);
        A = jacobian.transpose() * jacobian;
        ROS_DEBUG( "K<%d,%d> sum:%lf, sum2:%lf", jacobian.rows(), jacobian.cols(), jacobian.sum(), A.sum() );
        _J.push_back(jacobian);
        _A.push_back(A);
        _roimask.push_back(roimask);
    }

    isJacobiansAvailable = true;



    //sOverlay(_im_r_color[2], _roimask[2]);

    ROS_DEBUG( "L<%d,%d> sum:%lf, sum2:%lf", _J[0].rows(), _J[0].cols(), _J[0].sum(), _A[0].sum() );
    ROS_DEBUG( "L<%d,%d> sum:%lf, sum2:%lf", _J[1].rows(), _J[1].cols(), _J[1].sum(), _A[1].sum() );
    ROS_DEBUG( "L<%d,%d> sum:%lf, sum2:%lf", _J[2].rows(), _J[2].cols(), _J[2].sum(), _A[2].sum() );
    ROS_DEBUG( "L<%d,%d> sum:%lf, sum2:%lf", _J[3].rows(), _J[3].cols(), _J[3].sum(), _A[3].sum() );


}

/// @brief For a given pyramid level to compute itz semi-dense jacobian
/// The jacobian is computed as described in DVO paper. Jacobian is only computed (and stacked) at points which have higher image gradients.
/// These semi-dense keypoint locations are also stored.
/// @param[in] level : The pyramid level for which to compute jacobian
/// @param[out] J : This can be initially empty or non empty. This will be filled up in this function. Will have 6 cols
/// @param[out] semiDenseMarkings : A matrix (have values 0s & 1s) of the size of the image
/// @note The jacobian is only computed for the reference frame
void RGBDOdometry::computeJacobian(int level, Eigen::MatrixXd& J, Eigen::MatrixXi& semiDenseMarkings)
{
    assert( level >= 0  && level < 4 && "Level has to be between 0 and 4");

    assert( isPyramidalRefFrameAvailable && isRefFrameAvailable );


    // Get the image reference ptr from the pyramid
    cv::Mat& j_im = _im_r[level];
    cv::Mat& j_im_color = _im_r_color[level];
    cv::Mat& j_dim = _dim_r[level];


    ROS_INFO( "Jacobian Computation begins:(Level:%d)  %dx%d", level, j_im.rows, j_im.cols);


    // Kernerls for grad computation
    cv::Mat kernX = (cv::Mat_<float>(3,3) <<  0, 0,  0,
            0,  -1.0, 1.0,
            0, 0,  0);
    cv::Mat kernY = (cv::Mat_<float>(3,3) <<  0, 0,  0,
            0,  -1.0, 0,
            0, 1.0,  0);


    // Filtering
    cv::Mat imgx, imgy;
    cv::filter2D( j_im, imgx,CV_64F,kernX );
    cv::filter2D( j_im, imgy,CV_64F,kernY );

//    cv::Mat tmpo;
//    gx.convertTo(tmpo, CV_8UC1);
//    cv::imshow( "gradx", tmpo );
//    cv::imshow("depth", j_dim);

    // Convert to Eigen::MatrixXd
    Eigen::MatrixXd egx, egy,depth;
    cv::cv2eigen(imgx, egx);
    cv::cv2eigen(imgy, egy);

    cv::cv2eigen(j_dim, depth);


    semiDenseMarkings = Eigen::MatrixXi::Zero(j_dim.rows,j_dim.cols);



    assert( cameraIntrinsicsReady );

    Eigen::MatrixXd tJ(6,const_maxJacobianSize);

    // Only compute jacobians at pixels which have higher image gradient
    int xc=0;
    for( int j=0 ; j<egx.cols() ; j++ ) //since, eigen stores values in col-major format as opposite to row-major by opencv
    {
        for( int i=0 ; i<egx.rows() ; i++ )
        {
            assert( xc < const_maxJacobianSize );

            if( egx(i,j) < (double)const_gradientThreshold )
                continue;

            // mark pt
            semiDenseMarkings(i,j) = 1;


            double X,Y,Z;
            Z = depth(i,j);
            X = Z* (i - cx)/fx;
            Y = Z* (j - cy)/fy;

            double invZ = 1/Z;
            double invZ2 = 1/(Z*Z);

            double gx, gy;
            gx = egx(i,j);
            gy = egy(i,j);

            tJ(0,xc) = fx*fx*invZ;
            tJ(1,xc) = fy*gy*invZ;
            tJ(2,xc) = -fy*gy*Y*invZ2 - fx*gx*X*invZ2;
            tJ(3,xc) = gy*( -fy*Y*Y*invZ2 - fy) - fx*gx*X*Y*invZ2;
            tJ(4,xc) = gx*(fx*X*X*invZ2 + fx) + fx*gy*X*Y*invZ2;
            tJ(5,xc) = fy*gy*X*invZ - fx*gy*Y*invZ;

            xc++;
        }
    }




    assert( xc > const_minimumRequiredPts  && "Bad image....too few points with good texture" );
    J = tJ.leftCols(xc).transpose(); //this J is returned
    ROS_INFO( "Computation of J<%d,%d> complete", J.rows(), J.cols() );






}


/// @brief Now that Jacobians, ref, now frames are available, this function does a gauss-newton iteration
/// @param[in] level : The pyramid level at which to perform the Gauss-Newton Iterations
/// @param[in,out] T : Input the initial starting point for the iterations. At the end will contain the optimized solutions.
void RGBDOdometry::gaussNewtonIterations( int level,  TransformRep& T)
{
    assert( isPyramidalRefFrameAvailable && isPyramidalNowFrameAvailable && isJacobiansAvailable );
    assert( level >= 0  && level < 4 && "Level has to be between 0 and 4");
    assert( (level != 0) && "Critical error, jacobians at level-0 (base) are not computed for complexity reasons");

    ROS_INFO( "Image (level:%d) : %d x %d", level, _im[level].rows, _im[level].cols );
    ROS_INFO( "J<%d,%d>", _J[level].rows(), _J[level].cols() );


    //
    // Init solution
    //
    ROS_INFO( "Gauss-Netwon Iterations begin");
    //Eigen::Transform<double, 3, Eigen::Affine> T;
    //TransformRep T;

    //ROS_INFO_STREAM( "init T(rot):\n"<< T.topLeftCorner(3,3) );
    //ROS_INFO_STREAM( "init T(trans):\n"<< T.rightCols(1).top(3) );
    ROS_INFO_STREAM( "init T(rot):\n"<< T.rotation() );
    ROS_INFO_STREAM( "init T(trans):\n"<< T.translation() );

    //
    // Retrive J, A
    //
    Eigen::MatrixXd J = _J[level];
    Eigen::MatrixXd A = _A[level];

    ROS_INFO_STREAM( "A = [ "<< A << "]");


    for( int itr=0 ; itr<3 ; itr++ )
    {
        //
        // Compute Epsilon (Warping)
        //
        Eigen::VectorXd epsilon;
        Eigen::MatrixXi newlocs;
        computeEpsilon(level, T, epsilon, newlocs );
        ROS_INFO( "Iteration %d ^^^^^^ epsilon<%d,%d>", itr, epsilon.rows(), epsilon.cols() );
        ROS_INFO_STREAM( "epsilon (norm="<< epsilon.norm() << ") "<< epsilon.head(5).transpose() << "..." << epsilon.tail(5).transpose());

        if( epsilon.norm() < 200.0 )
            break;


        ROS_INFO_STREAM( "T:\n"<< T.matrix() );


        //
        // Solve : A * \psi = - J'*epsilon
        //
        Eigen::VectorXd b = -J.transpose() * epsilon;

        Eigen::VectorXd psi = A.colPivHouseholderQr().solve(b);
        ROS_INFO_STREAM( "psi : "<< psi.transpose() );


        //
        // Update transformation with differencial-twist
        //
        Eigen::Matrix4d outTr;
        exponentialMap(psi, outTr);
        ROS_INFO_STREAM( "outTr:\n"<< outTr.matrix() );

        T = T * outTr.inverse();

        if( level==2 )
            sOverlay( _im_color[level], newlocs);
    }


    ROS_INFO_STREAM( "final T(rot):\n"<< T.rotation() );
    ROS_INFO_STREAM( "final T(trans):\n"<< T.translation() );
    //ROS_INFO_STREAM( "final T(rot):\n"<< T.topLeftCorner(3,3) );
    //ROS_INFO_STREAM( "final T(trans):\n"<< T.rightCols(1).top(3) );







}


/// @brief Computes the epsilon after warping with T
/// Does the warping of the ref frame wrt to now frame and writes epsilon
void RGBDOdometry::computeEpsilon(int level, TransformRep T, Eigen::VectorXd& epsilon, Eigen::MatrixXi& newroimask)
{
    ROS_DEBUG( "Epsilon computation starts");

    assert( isPyramidalRefFrameAvailable && isPyramidalNowFrameAvailable && isJacobiansAvailable );
    assert( level >= 0  && level < 4 && "Level has to be between 0 and 4");
    assert( (level != 0) && "Critical error, jacobians at level-0 (base) are not computed for complexity reasons");


    //
    // Load from pyramid
    //
    // Jacobian
    Eigen::MatrixXd p_J = _J[level];
    Eigen::MatrixXd p_A = _A[level];
    Eigen::MatrixXi p_roimask = _roimask[level];


    // now image
    cv::Mat& p_im = _im[level];
    cv::Mat& p_im_color = _im_color[level];
    cv::Mat& p_dim = _dim[level];

    // ref image
    cv::Mat& r_im = _im_r[level];
    cv::Mat& r_im_color = _im_r_color[level];
    cv::Mat& r_dim = _dim_r[level];

    Eigen::MatrixXd re_depth, re_gray, noe_gray;
    cv::cv2eigen(r_dim, re_depth);
    cv::cv2eigen(r_im, re_gray);
    cv::cv2eigen(p_im, noe_gray);



    ROS_DEBUG( "roi sum: %d", p_roimask.sum());
    assert(  (p_roimask.sum() == p_J.rows())  &&  "Dimension mismatch"  );


    //
    // Computation of Epsilon
    //
    // 1. transform the selected points of ref image using T
    // 2. project these newly created points (note:these will b new set of selected points
    // 3. diff of these pts & new-selected positions in now image := epsilon
    epsilon = Eigen::VectorXd::Zero(p_J.rows());
    newroimask = Eigen::MatrixXi::Zero(p_roimask.rows(), p_roimask.cols());

    int lCount=0;
    for( int j=0 ; j<p_roimask.cols() ; j++ )
    {
        for( int i=0 ; i<p_roimask.rows() ; i++ )
        {
            if( p_roimask(i,j) != 1 )
                continue;

            // get 3d points at these locations of ref image
            double X,Y,Z;
            Z = re_depth(i,j);
            X = Z* (i - cx)/fx;
            Y = Z* (j - cy)/fy;
            Eigen::Vector3d hX;
            hX << X,Y,Z;

            // transform
            Eigen::Vector3d out = T.inverse() * hX;



            //projection of these points
            double outu = out(0)*fx/out(2) + cx;
            double outv = out(1)*fy/out(2) + cy;



            ROS_INFO_ONCE( "original (u,v) : (%d,%d)", i, j );
            ROS_INFO_ONCE( "X:%.3f, Y:%.3f, Z:%.3f",X,Y,Z);
            ROS_INFO_STREAM_ONCE( "hX:\n"<< hX );
            ROS_INFO_STREAM_ONCE( "out:\n"<< out );

            ROS_INFO_ONCE( "original (u,v) : (%.3f,%.3f)", outu, outv );

            if( outu >= 0  && outu < noe_gray.rows() && outv >= 0 && outv < noe_gray.cols() )
            {
                epsilon(lCount) = re_gray(i,j) - noe_gray( floor(outu), floor(outv) );
                newroimask( floor(outu), floor(outv) ) = 1;
            }

            lCount++;

        }
    }


    ROS_DEBUG( "Epsilon computation complete");
    ROS_DEBUG_STREAM( "epsilon (glimpse)"<< epsilon.head(5).transpose() << "..." << epsilon.tail(5).transpose());


}








/// \brief Given a 6-DOF psi, does an exponential Map.
///
/// @param[in] w: Psi 6-vector
/// @param[out] : output transformation matrix
void RGBDOdometry::exponentialMap(Eigen::VectorXd& psi, Eigen::Matrix4d& outTr) {
    assert(psi.rows() == 6 && "PSI does not seem to have 6 rows");

    Eigen::Vector3d t = psi.head(3);
    Eigen::Vector3d w = psi.tail(3);



    Eigen::Matrix3d wx;
    to_se_3(w,wx);


    double theta = w.norm();
    ROS_DEBUG( "THETA = %lf", theta );
    if( theta < 1E-12 )
    {
        outTr = Eigen::Matrix4d::Identity();
        return;
    }

    Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
    Eigen::RowVector3d zer_row = Eigen::RowVector3d::Zero();


    Eigen::Matrix3d xR = I3 + sin(theta)/theta * wx + (1.0-cos(theta))/(theta*theta) * wx * wx;

    Eigen::Matrix3d xV = I3 + (1-cos(theta))/(theta*theta) * wx + (theta-sin(theta))/(theta*theta*theta) * wx *wx;


    outTr << xR, xV*t, zer_row, 1.0;



}

/// \brief Converts a vector to its se(3) matrix
///
/// @param[in] w : a 3 vector. May not be unit vector
/// @param[out] wx : se(3) of w
/// \note
/// Helper for `exponentialMap()`
void RGBDOdometry::to_se_3(Eigen::Vector3d& w, Eigen::Matrix3d& wx) {
    wx = Eigen::Matrix3d::Zero();

    wx(1,2) = -w(0);
    wx(0,2) =  w(1);
    wx(0,1) = -w(2);

    wx(2,1) =  w(0);
    wx(2,0) = -w(1);
    wx(1,0) =  w(2);
}

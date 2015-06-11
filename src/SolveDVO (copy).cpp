#include <SolveDVO.h>


/// default constructor. Init the subscribers
SolveDVO::SolveDVO()
{
    isCameraIntrinsicsAvailable = false;
    isFrameAvailable = false;
    isRefFrameAvailable = false;
    isNowFrameAvailable = false;

    grad_thresh = 10;


    //sub = nh.subscribe( "odometry/rgbd", 2, &SolveDVO::imageArrivedCallBack, this );
    sub = nh.subscribe( "Xtion/rgbdPyramid", 2, &SolveDVO::imageArrivedCallBack, this );

    pub_pc = nh.advertise<sensor_msgs::PointCloud>( "dvo/pointCloud", 1 );
    pub_final_pose = nh.advertise<geometry_msgs::PoseStamped>( "dvo/finalPose", 1 );

}




/// Input opencv XML file containing camera internal params and distortion coif.
/// Note: At this moment the distortion coif. are not used.
void SolveDVO::setCameraMatrix(const char *calibFile)
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
    fx = (float)cameraMatrix.at<double>(0,0);
    fy = (float)cameraMatrix.at<double>(1,1);
    cx = (float)cameraMatrix.at<double>(0,2);
    cy = (float)cameraMatrix.at<double>(1,2);

    K = Eigen::Matrix3f::Zero();
    K(0,0) = fx;
    K(1,1) = fy;
    K(0,2) = cx;
    K(1,2) = cy;
    K(2,2) = 1.0;

    K_inv = K.inverse();

    ROS_INFO( "[SolvePnP::setCameraMatrix] Camera Matrix & Distortion Coif Loaded");
    ROS_INFO( "fx=%.4f, fy=%.4f, cx=%.4f, cy=%.4f", fx, fy, cx, cy );


    isCameraIntrinsicsAvailable = true;

}




/// @brief Display an Eigen::MatrixXd as an image (using opencv imshow())
void SolveDVO::imshowEigenImage(const char *winName, Eigen::MatrixXd eim)
{
    cv::Mat tmp, tmpuint;
    cv::eigen2cv( eim, tmp );
    tmp.convertTo(tmpuint, CV_8UC1);

    cv::imshow(winName, tmpuint );
}

/// @brief Display an Eigen::MatrixXf as an image (using opencv imshow())
void SolveDVO::imshowEigenImage(const char *winName, Eigen::MatrixXf eim)
{
    cv::Mat tmp, tmpuint;
    cv::eigen2cv( eim, tmp );
    tmp.convertTo(tmpuint, CV_8UC1);

    cv::imshow(winName, tmpuint );
}

void SolveDVO::loadFromFile(const char *xmlFileName)
{
    cv::FileStorage fs( xmlFileName, cv::FileStorage::READ );
    ROS_INFO_STREAM( "Loading : "<< xmlFileName );

    this->rcvd_framemono.clear();
    this->rcvd_depth.clear();
    for( int i=0 ; i<4 ; i++ )
    {
        cv::Mat framemono, dframe;
        char matmonoName[100], matdepthName[100];
        sprintf( matmonoName, "mono_%d", i );
        fs[matmonoName] >> framemono;

        sprintf( matdepthName, "depth_%d", i );
        fs[matdepthName] >> dframe;


        Eigen::MatrixXf eframemono, edframe;
        cv::cv2eigen( framemono, eframemono );
        cv::cv2eigen( dframe, edframe );

        ROS_DEBUG_STREAM( "loaded `"<< matmonoName << "` : "<< eframemono.rows() << ", " << eframemono.cols() );
        ROS_DEBUG_STREAM( "loaded `"<< matdepthName << "` : "<< edframe.rows() << ", " << edframe.cols() );

        rcvd_framemono.push_back(eframemono);
        rcvd_depth.push_back(edframe);
    }
    isFrameAvailable = true;

}



/// @brief Callback to receive RGBD from the topic "Xtion/rgbdPyramid"
///
/// The subscriber is defined in the construction. This function is the callback for the same.
/// This is a blocking function. Thus do not do any processing here.
void SolveDVO::imageArrivedCallBack( rgbd_odometry::RGBDFramePydConstPtr msg )
{
    ROS_INFO_STREAM_ONCE( "1st RGBD frame received. Will continue receiving but not report anymore on this");
    isFrameAvailable=false;


    cv::Mat frame, dframe, framemono;
    Eigen::MatrixXf e_dframe, e_framemono;
    this->rcvd_framemono.clear();
    this->rcvd_depth.clear();
    try
    {
        ROS_INFO_ONCE( "[imageArrivedCallBack] %lu images available", msg->dframe.size() );

        for( int i=0 ; i<msg->dframe.size() ; i++ ) //loop over the image pyramid (typically 4)
        {
            // retrive frames from message
            frame = cv_bridge::toCvCopy(msg->framergb[i], "bgr8")->image;
            framemono =  cv_bridge::toCvCopy(  msg->framemono[i], "mono8" )->image ;
            dframe =  cv_bridge::toCvCopy(  msg->dframe[i], "mono16" )->image ;

            dframe.setTo(1, (dframe==0) ); //to avoid zero depth


            // convert to Eigen
            cv::cv2eigen(framemono, e_framemono);
            cv::cv2eigen(dframe, e_dframe);

            // push onto vector
            this->rcvd_framemono.push_back(e_framemono);
            this->rcvd_depth.push_back(e_dframe);
        }

    }
    catch( cv_bridge::Exception& e )
    {
        ROS_ERROR( "cv_bridge exception: %s", e.what() );
        isFrameAvailable = false;
        return;
    }

    isFrameAvailable=true;

}

/// @brief Sets the rcvd frame as reference frame
void SolveDVO::setRefFrame()
{
    assert( isFrameAvailable );
    assert( rcvd_framemono.size() > 0  && rcvd_depth.size() > 0 );

    isRefFrameAvailable = false;
    J_r.clear();
    ROI_r.clear();
    im_r = rcvd_framemono;
    dim_r = rcvd_depth;
    isRefFrameAvailable = true;
}


/// @brief Sets the rcvd frame as now frame
void SolveDVO::setNowFrame()
{
    assert( isFrameAvailable );
    assert( rcvd_framemono.size() > 0  && rcvd_depth.size() > 0 );

    isNowFrameAvailable = false;
    im_n = rcvd_framemono;
    dim_n = rcvd_depth;
    isNowFrameAvailable = true;
}


/// @brief Computes the Jacobian matrix of the reference frame
/// @note ROI & J are the results
void SolveDVO::computeJacobian(int level, JacobianList& J, Eigen::MatrixXi& roi )
{
    assert( level >= 0 && level <= 3 );
    assert( isRefFrameAvailable ); //because jacobian is computed at reference frame (inverse formulation)
    ROS_INFO( "Start Computation of Jacobian at level=%d", level );

    float scaleFac = (float)pow(2,-level);

    Eigen::MatrixXf& _im = this->im_r[level];
    Eigen::MatrixXf& _dim = this->dim_r[level];


    //
    // Step - 1 : Computation of Image Gradient
    Eigen::MatrixXf Gx, Gy;
    imageGradient( _im, Gx, Gy );



    roi = Eigen::MatrixXi::Zero(_im.rows(), _im.cols());
    Eigen::RowVector2f G;
    Eigen::MatrixXf A1, A2;
    A1 = Eigen::MatrixXf::Zero(2,3);
    A2 = Eigen::MatrixXf::Zero(3,6);

    // no more needed to precompute number of selected points
    int nSelectPt;
//    nSelectPt = countSelectedPts(Gx,Gy);
//    ROS_INFO_STREAM( "# of Pts selected : "<< nSelectPt );


    J.clear(); // clear the list (just a precaution)
    J.push_back(Eigen::RowVectorXf::Zero(6)); // (1st J is always junk, this is done because we want to index these J_i onto the ROI

    nSelectPt = 0;
    for( int xx=0 ; xx<Gx.cols() ; xx++ )
    {
        for( int yy=0 ; yy<Gx.rows() ; yy++ )
        {
            //if(   (Gx(yy,xx)*Gx(yy,xx) + Gy(yy,xx)*Gy(yy,xx))  >  grad_thresh   )
            if(   GRAD_NORM( Gx(yy,xx), Gy(yy,xx) ) >  (grad_thresh)   )
            {
                // this can be speeded up by computing Jacobian vector in closed form for a point
                G(0) = Gx(yy,xx);
                G(1) = Gy(yy,xx);

                //
                // Step-2 : At selected points, get A1
                float Z = _dim(yy,xx);
                float X = Z * (xx-scaleFac*cx) / fx;
                float Y = Z * (yy-scaleFac*cy) / fy;

                A1(0,0) = fx*1/Z;
                A1(0,1) = 0.;
                A1(0,2) = -fx*X/(Z*Z);
                A1(1,0) = 0.;
                A1(1,1) = fy*1/Z;
                A1(1,2) = -fy*Y/(Z*Z);


                //
                // Step-3 : At selected points, get A2
                //A2.topLeftCorner(3,3) = -Eigen::MatrixXf::Identity(3,3);
                A2.block<3,3>(0,0) = -Eigen::MatrixXf::Identity(3,3);
                Eigen::Matrix3f tmpwx;
                to_se_3( X,Y,Z, tmpwx );
                //A2.topRightCorner(3,3) = tmpwx;
                A2.block<3,3>(0,3) = tmpwx;


                //
                // Step-4 : Multiply above 3 matrices to form a 1x6 matrix at selected points
                Eigen::RowVectorXf J_i = G * A1 * A2;
                //for( int k=0 ; k<6 ; k++ )
                //    J(nSelectPt,k) = J_i(k);
                J.push_back(J_i);



                //roi(yy,xx) = 1;
                roi(yy,xx) = J.size() - 1; //index of the stored J_i
                nSelectPt++;


            }

        }
    }

    ROS_INFO_STREAM( "Jacobian computed at : "<< nSelectPt << " pixel locations" );


    ROS_INFO( "End of Jacobian Computation");


}

void SolveDVO::gaussNewtonIterations(int level, int maxIterations, Eigen::Matrix3f& cR, Eigen::Vector3f& cT)
{
    assert( level >= 0 && level <= 3 );
    assert( maxIterations > 0 );
    assert( isRefFrameAvailable && isNowFrameAvailable && isCameraIntrinsicsAvailable );


    ROS_INFO( "Start of Gauss-Newton Iterations" );

    ROS_INFO_STREAM( "init R :\n"<< cR );
    ROS_INFO_STREAM( "init T :\n"<< cT );

    for( int itr = 0 ; itr < maxIterations ; itr++ )
    {
        ROS_INFO( "======== Iteration - %d ========", itr );
        Eigen::MatrixXf A;
        Eigen::VectorXf b;
        ROS_INFO( "starts computeEpsilon() ");
        bool flag = computeEpsilon( level, cR, cT, A, b );
        ROS_INFO( "end computeEpsilon()");

        if( flag == false )
            break;

        ROS_INFO( "start solve linear system");
        // solve equation
        Eigen::VectorXf psi = A.colPivHouseholderQr().solve(b);
        ROS_INFO( "end solve linear system");


        ROS_INFO( "start update estimates");
        // twists to R,T matrix
        Eigen::Matrix3f xRot;
        Eigen::Vector3f xTrans;
        exponentialMap( psi, xRot, xTrans );

        // update
        cT = xRot.transpose() * ( cT - xTrans );
        cR = xRot.transpose() * cR;
        //cT = cR * xRot.transpose() * xTrans + cT;
        //cR = cR * xRot.transpose();
        ROS_INFO_STREAM( "updated R :\n"<< cR );
        ROS_INFO_STREAM( "updated T :\n"<< cT );
        ROS_INFO( "end update estimates");

    }

    ROS_INFO_STREAM( "final estimate R :\n"<< cR );
    ROS_INFO_STREAM( "final estimate T :\n"<< cT );

    ROS_INFO( "End of Gauss-Newton Iterations");

}


/* //old code computeEpsilon()
bool SolveDVO::computeEpsilon(int level, Eigen::Matrix3f &cR, Eigen::Vector3f &cT, Eigen::MatrixXf& A, Eigen::VectorXf& b)
{
    float scaleFac = (float)pow(2,-level);
    ROS_INFO( "Level : %d", level );

    ROS_INFO_STREAM( "R :\n"<< cR );
    ROS_INFO_STREAM( "T :\n"<< cT );

    Eigen::MatrixXf _ref = im_r[level];
    JacobianList _J_ref = J_r[level];

    Eigen::MatrixXf _im = im_n[level];
    Eigen::MatrixXf _depth = dim_n[level];
    Eigen::MatrixXf Gx, Gy;
    imageGradient( _im, Gx, Gy );

    Eigen::MatrixXi roi = Eigen::MatrixXi::Zero(Gx.rows(), Gx.cols());
    Eigen::MatrixXi roiRef = ROI_r[level];

    int nPts = 0, roiMatched=0;
    Eigen::Vector3f qn, qr;
    Eigen::Vector2f qr_pj;

    Eigen::RowVectorXf J_i;
    A = Eigen::MatrixXf::Zero(6,6);
    b = Eigen::VectorXf::Zero(6);

    ROS_INFO( "loop starts");
    for( int xx=0 ; xx<Gx.cols() ; xx++ )
    {
        for( int yy=0 ; yy<Gx.rows() ; yy++ )
        {

            if(   GRAD_NORM( Gx(yy,xx), Gy(yy,xx) ) >  grad_thresh   )
            {
                float Z = _depth(yy,xx);

                float X = Z * ((float)xx-scaleFac*cx) / fx;
                float Y = Z * ((float)yy-scaleFac*cy) / fy;
                //qn << X,Y,Z;
                qn(0) = X; qn(1) = Y; qn(2) = Z;


                qr = cR * ( qn + cT );
                //qr = qn;


                qr_pj( 0 ) = ( (fx * qr(0)/qr(2)) + scaleFac*cx );
                qr_pj( 1 ) = ( (fy * qr(1)/qr(2)) + scaleFac*cy );


                int u_proj = (int)floor( qr_pj(1) );
                int v_proj = (int)floor( qr_pj(0) );



                if( v_proj>=0 && v_proj<Gx.cols() && u_proj>=0 && u_proj<Gx.rows() )
                {
                    roi( u_proj, v_proj ) = 2;
                    nPts++;
                    //if( roiRef( u_proj, v_proj ) > 0 )
                    {
                        int matchedIndx = roiRef( u_proj, v_proj );
                        roiMatched++;
                        // J_i' * r;  J_i' * J_i
                        float r = _ref(u_proj, v_proj) - _im(yy,xx); //pixel-error
                        J_i = _J_ref[matchedIndx]; //corresponding jacobian (6-vec) at this pixel

                        A += J_i.transpose() * J_i;
                        b += -r*J_i.transpose();
                    }
                }



                //roi(yy,xx) = 1;
            }
        }
    }
    ROS_INFO( "loop ends");
    ROS_INFO( "[computeEpsilon] Points selected (from now image) : %d", nPts );
    ROS_INFO( "[computeEpsilon] matched ROI : %d", roiMatched );

    if( roiMatched < 50 )
    {
        ROS_WARN( "Too few matched points. Signaling for stopping further iterations" );
        return false;
    }

//    cv::Mat outImg;
//    sOverlay(_im, roi, outImg, cv::Vec3b(0,255,255) );
//    cv::imshow( "marked_image_ref", outImg );

    return true;

}
*/


/* //trying to change
bool SolveDVO::computeEpsilon(int level, Eigen::Matrix3f &cR, Eigen::Vector3f &cT, Eigen::MatrixXf& A, Eigen::VectorXf& b)
{
    float scaleFac = (float)pow(2,-level);
    ROS_INFO( "Level : %d", level );

    ROS_INFO_STREAM( "R :\n"<< cR );
    ROS_INFO_STREAM( "T :\n"<< cT );

    Eigen::MatrixXf _ref = im_r[level];
    JacobianList _J_ref = J_r[level];

    Eigen::MatrixXf _im = im_n[level];
    //Eigen::MatrixXf _depth = dim_n[level];
    Eigen::MatrixXf _depth = dim_r[level];
    Eigen::MatrixXf Gx, Gy;
    //imageGradient( _im, Gx, Gy );
    imageGradient( _ref, Gx, Gy );

    Eigen::MatrixXi roi = Eigen::MatrixXi::Zero(Gx.rows(), Gx.cols());
    Eigen::MatrixXi roiRef = ROI_r[level];

    int nPts = 0, roiMatched=0;
    Eigen::Vector3f qn, qr;
    Eigen::Vector2f qr_pj;

    Eigen::RowVectorXf J_i;
    A = Eigen::MatrixXf::Zero(6,6);
    b = Eigen::VectorXf::Zero(6);

    ROS_INFO( "loop starts");
    for( int xx=0 ; xx<Gx.cols() ; xx++ )
    {
        for( int yy=0 ; yy<Gx.rows() ; yy++ )
        {

            if(   GRAD_NORM( Gx(yy,xx), Gy(yy,xx) ) >  grad_thresh   )
            {
                float Z = _depth(yy,xx);

                float X = Z * ((float)xx-scaleFac*cx) / fx;
                float Y = Z * ((float)yy-scaleFac*cy) / fy;
                //qn << X,Y,Z;
                qn(0) = X; qn(1) = Y; qn(2) = Z;


                //qr = cR * ( qn + cT );
                qr = cR.transpose() * ( qn - cT );
                //qr = qn;


                qr_pj( 0 ) = ( (fx * qr(0)/qr(2)) + scaleFac*cx );
                qr_pj( 1 ) = ( (fy * qr(1)/qr(2)) + scaleFac*cy );


                int u_proj = (int)floor( qr_pj(1) );
                int v_proj = (int)floor( qr_pj(0) );



                if( v_proj>=0 && v_proj<Gx.cols() && u_proj>=0 && u_proj<Gx.rows() )
                {
                    roi( u_proj, v_proj ) = 2;
                    nPts++;
                    //if( roiRef( u_proj, v_proj ) > 0 )
                    {
                        //int matchedIndx = roiRef( u_proj, v_proj );
                        int matchedIndx = roiRef( yy,xx );
                        roiMatched++;
                        // J_i' * r;  J_i' * J_i
                        //float r = _ref(u_proj, v_proj) - _im(yy,xx); //pixel-error
                        float r = _ref(yy,xx) - _im(u_proj, v_proj);
                        J_i = _J_ref[matchedIndx]; //corresponding jacobian (6-vec) at this pixel

                        A += J_i.transpose() * J_i;
                        b += -r*J_i.transpose();
                    }
                }



                //roi(yy,xx) = 1;
            }
        }
    }
    ROS_INFO( "loop ends");
    ROS_INFO( "[computeEpsilon] Points selected (from now image) : %d", nPts );
    ROS_INFO( "[computeEpsilon] matched ROI : %d", roiMatched );

    if( roiMatched < 50 )
    {
        ROS_WARN( "Too few matched points. Signaling for stopping further iterations" );
        return false;
    }

//    cv::Mat outImg;
//    sOverlay(_im, roi, outImg, cv::Vec3b(0,255,255) );
//    cv::imshow( "marked_image_ref", outImg );

    return true;

}
*/


bool SolveDVO::computeEpsilon(int level, Eigen::Matrix3f &cR, Eigen::Vector3f &cT, Eigen::MatrixXf& A, Eigen::VectorXf& b)
{
    float scaleFac = (float)pow(2,-level);

    Eigen::MatrixXf _ref = im_r[level];
    Eigen::MatrixXf _depth = dim_r[level];
    Eigen::MatrixXi _ROI_r = ROI_r[level];
    JacobianList _J_ref = J_r[level];
    Eigen::MatrixXf _now = im_n[level];

    //some init
    Eigen::RowVectorXf J_i;
    A = Eigen::MatrixXf::Zero(6,6);
    b = Eigen::VectorXf::Zero(6);


    //loop over all the selected points from ref image
    ROS_INFO( "loop starts");
    int nPtTransformed = 0;
    int nPtVisible = 0;
    for( int xx=0 ; xx<_ref.cols() ; xx++ )
    {
        for( int yy=0 ; yy<_ref.rows() ; yy++ )
        {

            if( _ROI_r(yy,xx) > 0 )
            {
                //get 3d pts at selected pts (u,v)
                float Z = _depth(yy,xx);

                float X = Z * ((float)xx-scaleFac*cx) / fx;
                float Y = Z * ((float)yy-scaleFac*cy) / fy;
                Eigen::Vector3f pr(X,Y,Z);

                //change frame-of-ref of these 3d pts.
                Eigen::Vector3f pn = cR.transpose() * ( pr - cT );
                nPtTransformed++;


                // project above pt to obtain (u_proj, v_proj)
                int u_proj = (int) floor( fx*(pn(0) / pn(2)) + scaleFac*cx );
                int v_proj = (int) floor( fy*(pn(1) / pn(2)) + scaleFac*cy );


                if( v_proj>=0 && v_proj<_now.rows() && u_proj>=0 && u_proj<_now.cols() )
                {
                    nPtVisible++;
                    // read I_n(u_proj, v_proj).
                    //      compute : r:= I_r(u,v) -  I_n(u_proj, v_proj)
                    //      compute : J' * J
                    //      compute : J' * r
                    int matchedIndx = _ROI_r( yy, xx );
                    assert( (matchedIndx > 0)  && (matchedIndx<_J_ref.size()) );


                    float r = _ref(yy,xx) - _now(v_proj, u_proj  );


                    J_i = -_J_ref[matchedIndx]; //corresponding jacobian (6-vec) at this pixel

                    A += J_i.transpose() * J_i;
                    b += -r*J_i.transpose();

                }


            }
        }
    }
    ROS_INFO( "loop ends");
    ROS_INFO( "Points Transformed : %d", nPtTransformed );
    ROS_INFO( "Points (of ref-frame) visible in now-frame : %d", nPtVisible);
    ROS_WARN_COND( nPtVisible<50, "Too few visible points");

    if( nPtVisible < 50 )
        return false;
    else
        return true;
}

/// @brief Computation of Image gradient along X and along Y
/// Computation of Image gradient along X and along Y. For now the input image (in Eigen format) is converted to Opencv.
/// The gradient is computed using opencv functions and results are converted back to Eigen format.
/// @param[in] image : 1-channel input image of which to find gradient
/// @param[out] gradientX : Resulting image gradient along X-axis
/// @param[out] gradientY : Resulting image gradient along Y-axis
void SolveDVO::imageGradient(Eigen::MatrixXf& image, Eigen::MatrixXf &gradientX, Eigen::MatrixXf &gradientY )
{
    // convert to opencv and do image gradient
    // this is fast enough for now...
    cv::Mat tmpIm32f;
    cv::eigen2cv( image, tmpIm32f );
    // Kernerls for grad computation
    cv::Mat kernX = (cv::Mat_<float>(3,3) <<  0, 0,  0,
            0,  -1.0, 1.0,
            0, 0,  0);
    cv::Mat kernY = (cv::Mat_<float>(3,3) <<  0, 0,  0,
            0,  -1.0, 0,
            0, 1.0,  0);



    // Filtering
    cv::Mat imgx, imgy;
    cv::filter2D( tmpIm32f, imgx,CV_32F,kernX );
    cv::filter2D( tmpIm32f, imgy,CV_32F,kernY );

    cv::cv2eigen( imgx, gradientX );
    cv::cv2eigen( imgy, gradientY );



    // Direct Implement if necessary....
}


/// \brief Converts a vector to its se(3) matrix
/// @param[in] w : a 3 vector. May not be unit vector
/// @param[out] wx : se(3) of w
void SolveDVO::to_se_3(Eigen::Vector3f& w, Eigen::Matrix3f& wx) {
    wx = Eigen::Matrix3f::Zero();

    wx(1,2) = -w(0);
    wx(0,2) =  w(1);
    wx(0,1) = -w(2);

    wx(2,1) =  w(0);
    wx(2,0) = -w(1);
    wx(1,0) =  w(2);
}


/// \brief Converts a vector to its se(3) matrix
/// @param[in] w(0) : 1st component of w
/// @param[in] w(1) : 2nd component of w
/// @param[in] w(2) : 3rd component of w
/// @param[out] wx : se(3) of w
void SolveDVO::to_se_3(float w0, float w1, float w2, Eigen::Matrix3f& wx) {
    wx = Eigen::Matrix3f::Zero();

    wx(1,2) = -w0;
    wx(0,2) =  w1;
    wx(0,1) = -w2;

    wx(2,1) =  w0;
    wx(2,0) = -w1;
    wx(1,0) =  w2;
}


/// \brief Given a 6-DOF psi, does an exponential Map.
///
/// @param[in] w: Psi 6-vector
/// @param[out] : output transformation matrix
void SolveDVO::exponentialMap(Eigen::VectorXf& psi, Eigen::Matrix3f& outR, Eigen::Vector3f& outT) {
    assert(psi.rows() == 6 && "PSI does not seem to have 6 rows");

    Eigen::Vector3f t = psi.head(3);
    Eigen::Vector3f w = psi.tail(3);



    Eigen::Matrix3f wx;
    to_se_3(w,wx);


    float theta = w.norm();
    ROS_DEBUG( "THETA = %lf", theta );
    if( theta < 1E-12 )
    {
        //outTr = Eigen::Matrix4d::Identity();
        outR = Eigen::Matrix3f::Identity();
        outT = Eigen::Vector3f::Zero();
        return;
    }

    Eigen::Matrix3f I3 = Eigen::Matrix3f::Identity();
    Eigen::RowVector3f zer_row = Eigen::RowVector3f::Zero();


    Eigen::Matrix3f xR = I3 + sin(theta)/theta * wx + (1.0-cos(theta))/(theta*theta) * wx * wx;

    Eigen::Matrix3f xV = I3 + (1-cos(theta))/(theta*theta) * wx + (theta-sin(theta))/(theta*theta*theta) * wx *wx;


    outR = xR;
    outT = xV*t;
    //outTr << xR, xV*t, zer_row, 1.0;
}

void SolveDVO::sOverlay( Eigen::MatrixXf eim, Eigen::MatrixXi mask, cv::Mat& xim, cv::Vec3d color )
{
    assert( (eim.rows() == mask.rows()) && "Image and mask rows must match");
    assert( (eim.cols() == mask.cols()) && "Image and mask cols must match");

    cv::Mat tmpIm, tmpIm8;
    cv::eigen2cv( eim, tmpIm );
    tmpIm.convertTo(tmpIm8, CV_8UC1);
    xim = cv::Mat::zeros(eim.rows(), eim.cols(), CV_8UC3 );
    std::vector<cv::Mat> ch;
    ch.push_back(tmpIm8);
    ch.push_back(tmpIm8);
    ch.push_back(tmpIm8);
    cv::merge(ch,xim);


    for( int j=0 ; j<mask.cols() ; j++ )
    {
        for( int i=0 ; i<mask.rows() ; i++ )
        {
            /*
            if( mask(i,j) == 1 )
            {
                xim.at<cv::Vec3b>(i,j)[0] = 0;
                xim.at<cv::Vec3b>(i,j)[1] = 0;
                xim.at<cv::Vec3b>(i,j)[2] = 255;
            }
            else if( mask(i,j) == 2 )
            {
                xim.at<cv::Vec3b>(i,j)[0] = 0;
                xim.at<cv::Vec3b>(i,j)[1] = 255;
                xim.at<cv::Vec3b>(i,j)[2] = 255;
            }
            */
            if( mask(i,j) > 0 )
                xim.at<cv::Vec3b>(i,j) = color;

        }
    }

}

int SolveDVO::countSelectedPts(Eigen::MatrixXf Gx, Eigen::MatrixXf Gy)
{
    int count=0;
    for( int xx=0 ; xx<Gx.cols() ; xx++ )
    {
        for( int yy=0 ; yy<Gx.rows() ; yy++ )
        {
            if(   GRAD_NORM( Gx(yy,xx), Gy(yy,xx) ) >  grad_thresh   )
            {
                count++;
            }
        }
    }
    return count;
}


/// @brief The event loop. Basically an ever running while with ros::spinOnce()
/// This is a re-implementation taking into care the memory scopes and also processing only points with higher image gradients
void SolveDVO::loop()
{


    ros::Rate rate(30);
    long nFrame=0;
    JacobianList J;
    Eigen::MatrixXi roi;
    Eigen::Matrix3f cR = Eigen::Matrix3f::Identity();
    Eigen::Vector3f cT = Eigen::Vector3f::Zero();
    while( ros::ok() )
    {
        ros::spinOnce();
        if( !(this->isFrameAvailable) )
            continue;


        if( (nFrame % 9000) == 0 )
        {
            setRefFrame();
            J_r.clear();
            ROI_r.clear();
            computeJacobian(0, J, roi);     J_r.push_back(J); ROI_r.push_back(roi); J.clear();
            computeJacobian(1, J, roi);     J_r.push_back(J); ROI_r.push_back(roi); J.clear();
            computeJacobian(2, J, roi);     J_r.push_back(J); ROI_r.push_back(roi); J.clear();
            computeJacobian(3, J, roi);     J_r.push_back(J); ROI_r.push_back(roi); J.clear();
            cR = Eigen::Matrix3f::Identity();
            cT = Eigen::Vector3f::Zero();
        }

        setNowFrame();
        gaussNewtonIterations( 2, 3, cR, cT );


        //publishBowl();
        //publishCurrentPointCloud();
        publishPose(cR, cT);



        imshowEigenImage("imNow", this->im_n[1]);
        imshowEigenImage("imRef1", this->im_r[1]);
        imshowEigenImage("imRef0", this->im_r[0]);

        cv::waitKey(3);

        rate.sleep();
        nFrame++;
    }



/*
    JacobianList J;
    Eigen::MatrixXi roi;

    loadFromFile( "xdump/framemono_0225.xml" );
    setRefFrame();
    J_r.clear(); ROI_r.clear();
    computeJacobian(0, J, roi);     J_r.push_back(J); ROI_r.push_back(roi); J.clear();
    computeJacobian(1, J, roi);     J_r.push_back(J); ROI_r.push_back(roi); J.clear();
    computeJacobian(2, J, roi);     J_r.push_back(J); ROI_r.push_back(roi); J.clear();
    computeJacobian(3, J, roi);     J_r.push_back(J); ROI_r.push_back(roi); J.clear();



    loadFromFile( "xdump/framemono_0226.xml" );
    setNowFrame();

    // Some displaying
    {

    cv::Mat outImg;
    sOverlay(im_r[0], ROI_r[0], outImg, cv::Vec3b(0,0,255) );
    cv::imshow( "marked_image", outImg );
    imshowEigenImage("imNow", this->im_n[1]);
    imshowEigenImage("imRef1", this->im_r[1]);
    imshowEigenImage("imRef0", this->im_r[0]);
    }



    Eigen::Matrix3f cR = Eigen::Matrix3f::Identity();
    Eigen::Vector3f cT = Eigen::Vector3f::Zero();

    gaussNewtonIterations( 0, 7, cR, cT );


    ros::Rate rate(30);
    while( ros::ok() )
    {
        ros::spinOnce();
        publishCurrentPointCloud();

        publishPose(cR, cT);

        cv::waitKey(3);
        rate.sleep();
    }
*/

}




void SolveDVO::publishBowl()
{
    sensor_msgs::PointCloud pcl_msg;
    pcl_msg.header.frame_id = "denseVO";
    pcl_msg.header.stamp = ros::Time::now();


    for( float x=-100 ;  x<100 ; x+=1.0 )
    {
        for( float y=-100 ; y<100 ; y+=1.0 )
        {
            float z = x*x + 20*y*y;
            geometry_msgs::Point32 pt;
            pt.x = x;
            pt.y = y;
            pt.z = z;

            pcl_msg.points.push_back(pt);
        }
    }
    pub_pc.publish( pcl_msg );
}


void SolveDVO::publishCurrentPointCloud()
{
    assert( isCameraIntrinsicsAvailable && isNowFrameAvailable );
    assert( im_n.size() > 1 );


    sensor_msgs::PointCloud pcl_msg;
    pcl_msg.header.frame_id = "denseVO";
    pcl_msg.header.stamp = ros::Time::now();


    int level = 1;
    Eigen::MatrixXf& _im = this->im_n[level];
    Eigen::MatrixXf& _dim = this->dim_n[level];

    sensor_msgs::ChannelFloat32 shade;
    shade.name = "intensity";


    for( int yy=0 ; yy<_im.rows() ; yy++ )
    {
        for( int xx=0 ; xx<_im.cols() ; xx++ )
        {
            float Z = _dim(yy,xx);
            if( Z < 10 )
                continue;
            float X = Z * (xx-.5*cx) / fx;
            float Y = Z * (yy-.5*cy) / fy;

            geometry_msgs::Point32 pt;
            pt.x = X;
            pt.y = Y;
            pt.z = Z;

            pcl_msg.points.push_back(pt);
            shade.values.push_back( _im(yy,xx) );
        }
    }

    pcl_msg.channels.push_back(shade);
    pub_pc.publish( pcl_msg );
}

void SolveDVO::publishPose(Eigen::MatrixXf rot, Eigen::VectorXf tran)
{
    geometry_msgs::Pose rospose;
    matrixToPose(rot, tran, rospose);

    geometry_msgs::PoseStamped poseS;
    poseS.header.frame_id = "denseVO";
    poseS.header.stamp = ros::Time::now();
    poseS.pose = rospose;

    pub_final_pose.publish( poseS );

}


/// @brief Given the rotation and translation matrix convert to ros Pose representation
/// @param[in] rot : 3x3 rotation matrix
/// @param[in] trans : 3-vector representing translation
/// @param[out] rosPose : geometry_msgs::Pose as output
void SolveDVO::matrixToPose(Eigen::Matrix3f rot, Eigen::Vector3f tran, geometry_msgs::Pose& rospose)
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







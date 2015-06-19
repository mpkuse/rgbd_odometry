#include <SolveDVO.h>


/// default constructor. Init the subscribers
SolveDVO::SolveDVO()
{
    isCameraIntrinsicsAvailable = false;
    isFrameAvailable = false;
    isRefFrameAvailable = false;
    isNowFrameAvailable = false;
    isJacobianComputed = false;

    signalGetNewRefImage = true;


    //
    // Setting up some global constants
    grad_thresh = 3;
    rviz_frame_id = "denseVO";
    ratio_of_visible_pts_thresh = 0.85;



    //
    // Setting up Publishers & Subscribers

    //sub = nh.subscribe( "odometry/rgbd", 2, &SolveDVO::imageArrivedCallBack, this );
    sub = nh.subscribe( "Xtion/rgbdPyramid", 10, &SolveDVO::imageArrivedCallBack, this );

    pub_pc = nh.advertise<sensor_msgs::PointCloud>( "dvo/pointCloud", 1 );
    pub_final_pose = nh.advertise<geometry_msgs::PoseStamped>( "dvo/finalPose", 1 );
    pub_pose_wrt_ref = nh.advertise<geometry_msgs::PoseStamped>( "dvo/poseWrtRef", 1 );



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
    im_r = rcvd_framemono;
    dim_r = rcvd_depth;
    isRefFrameAvailable = true;
    isJacobianComputed = false;
}


/// @brief Sets the rcvd frame as now frame
void SolveDVO::setNowFrame()
{
    assert( isFrameAvailable && "FRAME NOT AVAILABLE" );
    assert( rcvd_framemono.size() > 0  && rcvd_depth.size() > 0 );

    isNowFrameAvailable = false;
    im_n = rcvd_framemono;
    dim_n = rcvd_depth;
    isNowFrameAvailable = true;
}


/// @brief compute Jacobian for each level
void SolveDVO::computeJacobian()
{
    _J.clear();
    _imCord.clear();
    _spCord.clear();
    _intensities.clear();
    _roi.clear();
    for( int level=0 ; level<4 ; level++ )
    {
        JacobianList J;
        ImCordList ImC;
        SpaceCordList SpC;
        IntensityList inL;
        Eigen::MatrixXi roi;

        computeJacobian(level, J, ImC, SpC, inL, roi );

        //push_back everything on global variables
        _J.push_back(J);
        _imCord.push_back(ImC);
        _spCord.push_back(SpC);
        _intensities.push_back(inL);
        _roi.push_back(roi);
    }
    isJacobianComputed = true;
}


/// @brief Computes the Jacobian matrix of the reference frame
///
void SolveDVO::computeJacobian(int level, JacobianList& J, ImCordList& imC, SpaceCordList& spC, IntensityList& intensity, Eigen::MatrixXi& refROI )
{
    assert( level >= 0 && level <= 3 );
    assert( isRefFrameAvailable ); //because jacobian is computed at reference frame (inverse formulation)
    float scaleFac = (float)pow(2,-level);

    ROS_INFO( "Start Computation of Jacobian at level=%d", level );


    // retrive reference images (RGBD)
    Eigen::MatrixXf _ref = im_r[level];
    Eigen::MatrixXf _depth = dim_r[level];

    // Image gradient computation
    Eigen::MatrixXf Gx, Gy;
    imageGradient(_ref, Gx, Gy );





    int nGoodPts = countSelectedPts(Gx, Gy, refROI);
    ROS_INFO( "# Good Points : %d", nGoodPts );

    ROS_WARN_COND( (nGoodPts<50), "[computeJacobian] Too few interesting points to compute Jacobian at");


    // init the return values
    J.clear(); J.reserve(nGoodPts);
    imC = Eigen::MatrixXf::Zero(2,nGoodPts);
    spC = Eigen::MatrixXf::Zero(3,nGoodPts);
    //intensity.clear(); intensity.reserve(nGoodPts);
    intensity = Eigen::VectorXf::Zero(nGoodPts);
    //do not init refROI here. it is computed in countSelectedPts()


    //loop thru all the interesting points
    ROS_INFO( "loop begins");
    int nPtCount = 0;
    Eigen::MatrixXf A1 = Eigen::MatrixXf::Zero(2,3);
    Eigen::MatrixXf A2 = Eigen::MatrixXf::Zero(3,6);
    Eigen::RowVector2f G;


    float tmpfx = 1./(scaleFac*fx);
    float tmpfy = 1./(scaleFac*fy);
    float tmpcx = scaleFac*cx;
    float tmpcy = scaleFac*cy;

    for( int xx=0 ; xx<Gx.cols() ; xx++ )
    {
        for( int yy=0 ; yy<Gx.rows() ; yy++ )
        {
            float Z = _depth(yy,xx);
            if(   refROI(yy,xx) > 0   )
            {
                // compute 3d point (using depth) --(1)

                float X = Z * (xx-tmpcx) * tmpfx;
                float Y = Z * (yy-tmpcy) * tmpfy;

                float resduceFac = 1.0;
                X /= resduceFac; Y/=resduceFac; Z/=resduceFac;


                // compute Jacobian
                // G
                G(0) = Gx(yy,xx);
                G(1) = Gy(yy,xx);

                // A1
                A1(0,0) = scaleFac*fx/Z;
                A1(0,1) = 0.;
                A1(0,2) = -scaleFac*fx*X/(Z*Z);
                A1(1,0) = 0.;
                A1(1,1) = scaleFac*fy/Z;
                A1(1,2) = -scaleFac*fy*Y/(Z*Z);

                // A2
                A2.block<3,3>(0,0) = -Eigen::MatrixXf::Identity(3,3);
                Eigen::Matrix3f wx;
                to_se_3(X,Y,Z, wx );
                A2.block<3,3>(0,3) = wx;


                ROS_INFO_STREAM_ONCE( "(XYZ) : "<< X<< ","<< Y<< ", "<< Z << "\nA2 :\n"<< A2 );

                Eigen::RowVectorXf J_i = G * A1 * A2;



                // register 3d pts, im intensity, im cords, Jacobian (at this interesting pt) in the frame-of-ref of the reference frame
                spC(0,nPtCount) = resduceFac*X;
                spC(1,nPtCount) = resduceFac*Y;
                spC(2,nPtCount) = resduceFac*Z;

                //intensity.push_back( _ref(yy,xx) );
                intensity(nPtCount) = _ref(yy,xx);

                imC(0,nPtCount) = xx;
                imC(1,nPtCount) = yy;

                J.push_back(J_i);

                nPtCount++;
            }
        }
    }
    ROS_INFO( "loop ends");

    ROS_INFO( "# Image Cordinates : %d", nPtCount);
    ROS_INFO( "# Space Cordinates : %d", nPtCount);
    ROS_INFO( "# Ref Intensities  : %d", (int)intensity.size());
    ROS_INFO( "# Jacobians        : %d", (int)J.size());

    assert( (nPtCount == nGoodPts) && (nPtCount==(int)intensity.size()) && (intensity.size() == J.size()) );

    ROS_INFO("End of Jacobian Computation");
}

void SolveDVO::gaussNewtonIterations(int level, int maxIterations, Eigen::Matrix3f& cR, Eigen::Vector3f& cT)
{

    assert( level >= 0 && level <= 3 );
    assert( maxIterations > 0 );
    assert( isRefFrameAvailable && isNowFrameAvailable && isCameraIntrinsicsAvailable );


    ROS_INFO("-*-*-*-*- Start of Gauss-Newton Iterations (level=%d) -*-*-*-*-", level );

    ROS_DEBUG_STREAM( "init R :\n"<< cR );
    ROS_DEBUG_STREAM( "init T :\n"<< cT.transpose() );




    for( int itr = 0 ; itr < maxIterations ; itr++ )
    {
        ROS_INFO( "======== Iteration - %d ========", itr );
        ////////////////////////////////////////////////
        // Part - I : Compute Essilon
        ////////////////////////////////////////////////
        Eigen::MatrixXf A;
        Eigen::VectorXf b;
        ROS_DEBUG( "starts computeEpsilon() ");
        float ratio_of_visible_pts = computeEpsilon( level, cR, cT, A, b );
        ROS_DEBUG( "end computeEpsilon()");

        ROS_INFO( "ratio_of_visible_pts : %f", ratio_of_visible_pts );
        if( ratio_of_visible_pts < ratio_of_visible_pts_thresh )
        {
            //break;
            ROS_INFO( "ratio of visible points to available points is less than %f, signal getNewRefImage", ratio_of_visible_pts_thresh);
            signalGetNewRefImage = true;
            break;
        }



        ////////////////////////////////////////////////
        // Part - II : Solve Linear System of Equations
        ////////////////////////////////////////////////
        ROS_DEBUG( "start solve linear system");
        // solve equation
        //A = A + 1000000.0 * Eigen::MatrixXf::Identity(6,6);
        Eigen::VectorXf psi = A.colPivHouseholderQr().solve(b);
        ROS_DEBUG( "end solve linear system");
        ROS_INFO( "|psi| : %f", psi.norm());

//        ROS_DEBUG_STREAM( "A=\n["<< A << "]");
//        ROS_DEBUG_STREAM( "b:\n["<< b << "]");
        ROS_INFO_STREAM( "psi:\n["<< psi.transpose() << "]");



        ////////////////////////////////////////////////
        // Part - III : Update Estimates
        ////////////////////////////////////////////////
        ROS_DEBUG( "start update estimates");
        // twists to R,T matrix
        Eigen::Matrix3f xRot = Eigen::Matrix3f::Identity();
        Eigen::Vector3f xTrans = Eigen::Vector3f::Zero();
        //exponentialMap( psi, xRot, xTrans );

        // using sophus for expMap. Verified that the result from our method is exactly same as sophus
        Sophus::SE3f mat = Sophus::SE3f::exp(psi);
        xRot = mat.rotationMatrix();
        xTrans = mat.translation();
        ROS_DEBUG_STREAM( "Sophus R : \n["<< mat.rotationMatrix() << "]");
        ROS_DEBUG_STREAM( "Sophus T : \n["<< mat.translation() << "]");

        ROS_DEBUG_STREAM( "R_h:\n["<< xRot << "]");
        ROS_DEBUG_STREAM( "T_h:\n["<< xTrans << "]");


        printRT( cR, cT, "cR, cT before update" );



        updateEstimates( cR, cT, xRot, xTrans );


        printRT( cR, cT,  "cR, cT after update" );


        ROS_DEBUG_STREAM( "updated R :\n"<< cR );
        ROS_DEBUG_STREAM( "updated T :\n"<< cT.transpose() );


        ////////////////////////////////////////////////
        //                  DEBUG                     //
        ////////////////////////////////////////////////

        // writing the reprojections at each iterations
#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__
        if( level == __REPROJECTION_LEVEL )
        {
            visualizeResidueHistogram( __residues );
            visualizeResidueHeatMap(im_n[__REPROJECTION_LEVEL], __now_roi_reproj_values );
            visualizeReprojectedDepth(im_n[__REPROJECTION_LEVEL], __reprojected_depth);

        cv::Mat outImg2;
        sOverlay(im_n[__REPROJECTION_LEVEL], __now_roi_reproj, outImg2, cv::Vec3b(255,255,0) );
        cv::imshow( "DEBUG reprojected markers onto now", outImg2 );
        char key = cv::waitKey(0);
        if( key == 'b' )
            break;

        }
#endif //__SHOW_REPROJECTIONS_EACH_ITERATION__
        ROS_DEBUG( "end update estimates");

    }


    ROS_DEBUG_STREAM( "final R :\n"<< cR );
    ROS_DEBUG_STREAM( "final T :\n"<< cT.transpose() );
    ROS_INFO("End of Gauss-Newton Iterations" );


}


/// @brief Updates the current estimates of R_{t}, T_{t} using R_h, T_h ie. the best deltas from previous iteration
void SolveDVO::updateEstimates(Eigen::Matrix3f &cR, Eigen::Vector3f &cT, Eigen::Matrix3f &R_h, Eigen::Vector3f &T_h)
{
    /* old tries
    ////         update (derived by me)
    //        xRot.transposeInPlace();
    //        cT = xRot * ( cT - xTrans );
    //        cR = xRot * cR;

    // update (new derivation by me), this one is correct --1
    //        cT = xTrans + xRot.transpose() * cT;
    //        cR = cR.transpose() * xRot;

    // derivation -X (by me) --- works
    //        cT = -cR.transpose() * ( cT - xTrans );
    //        cR = xRot.transpose() * cR;
    //        cR.transposeInPlace();


    // X - but dont take inverse
    //        cT = xRot.transpose() * ( cT - xTrans );
    //        cR = xRot.transpose() * cR;

    // derivation -2 (by me)
    //        cT = xRot.transpose()*cT - xRot.transpose() * xTrans;
    //        cR = xRot.transpose() * cR;

    ////        // update as per SVO
    //        cT = xRot.transpose() * ( cT - xTrans );
    //        Eigen::Matrix3f tmpp = xRot.transpose() * cR;
    //        cR = tmpp;


    //        cT = xRot.transpose() * ( cT - xTrans );
    //        cR = xRot.transpose() * cR;



    //        cT = cR.transpose()*xRot.transpose()*xTrans - cR.transpose()*cT;
    //        Eigen::Matrix3f tmpp = cR.transpose() * xRot.transpose();
    //        cR = tmpp;


    //    cT = -cR.transpose() * cT;
    //    cR.transposeInPlace();
    //    cT = R_h.transpose() * ( cT - T_h);
    //    cR = R_h.transpose() * cR;
    */
    ///////////////////////////////////////////////// 2nd Innings ///////////////////////////////

    // T_cap * delta
//    cT = cR*T_h + cT;
//    cR = cR * R_h;


    // delta * T_cap.....perfect...!
    T_h = -R_h.transpose() * T_h;
    R_h.transposeInPlace();

    cT = R_h*cT + T_h;
    cR = R_h * cR;


}




// returns the ratio of visible points and total available points
float SolveDVO::computeEpsilon(int level, Eigen::Matrix3f &cR, Eigen::Vector3f &cT, Eigen::MatrixXf& A, Eigen::VectorXf& b)
{
    float scaleFac = (float)pow(2,-level);
    Eigen::Matrix3f scaleMatrix = Eigen::Matrix3f::Identity();
    scaleMatrix(0,0) = scaleFac;
    scaleMatrix(1,1) = scaleFac;


    SpaceCordList pts3d_ref = _spCord[level];
    ImCordList orgImCords = _imCord[level];
    IntensityList grays_ref = _intensities[level];
    JacobianList jacob = _J[level];

    Eigen::MatrixXf _now = im_n[level];

    A = Eigen::MatrixXf::Zero(6,6);
    b = Eigen::VectorXf::Zero(6);

    Eigen::MatrixXf cTRep;
    igl::repmat(cT,1,pts3d_ref.cols(),cTRep); // repeat cT col-wise (uses IGL library)
    Eigen::MatrixXf pts3d_n = cR.transpose() * ( pts3d_ref - cTRep );


#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__
    Eigen::MatrixXf pts3d_n_copy = pts3d_n; //note that this is a deep-copy

    if( level == __REPROJECTION_LEVEL )
    {
        __now_roi_reproj = Eigen::MatrixXi::Zero(_now.rows(), _now.cols()); //init to zero
        __residues = -Eigen::VectorXf::Ones( pts3d_ref.cols() ); // init to -1
        __now_roi_reproj_values = -Eigen::MatrixXf::Ones(_now.rows(), _now.cols()); //init to -1
        __reprojected_depth = -Eigen::MatrixXf::Ones( _now.rows(), _now.cols() );

    }
#endif //__SHOW_REPROJECTIONS_EACH_ITERATION__



    float cumm_r=0.0;


//    for( int i=0 ; i<3 ; i++ )
//        pts3d_ref.row(i).array() -= cT(i); //translation
//    Eigen::MatrixXf pts3d_n = cR.transpose() * pts3d_ref; // I have verified that this is exactly the same as `cR.transpose() * ( pts3d_ref - cTRep )`

    //convert to inhomogeneous
    Eigen::ArrayXXf lastRow_inv = pts3d_n.row(2).array().inverse();
    for( int i=0 ; i<3 ; i++ )
        pts3d_n.row(i).array() *= lastRow_inv;


    Eigen::MatrixXf pts3d_un_normalized = scaleMatrix * K * pts3d_n;


    int nPtVisible = 0;
    for( int i=0 ; i<pts3d_un_normalized.cols() ; i++ )
    {
        int xx = (int)pts3d_un_normalized(0,i);
        int yy = (int)pts3d_un_normalized(1,i);


        float r=0;


        if( xx >=0 && xx<_now.cols() && yy>= 0 && yy <_now.rows() )
        {


            r = grays_ref(i) - _now(yy,xx);

            float weight = getWeightOf( r );


            A += (jacob[i].transpose() * jacob[i])*weight;


#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__
            if( level == __REPROJECTION_LEVEL )
            {
                __now_roi_reproj(yy,xx) = 1;
                __now_roi_reproj_values(yy,xx) = (r>0)?r:-r;  // ===> absolute value of r
                __residues(i) = (r>0)?r:-r;
                __reprojected_depth(yy,xx) = pts3d_n_copy(2, i );
            }
#endif //__SHOW_REPROJECTIONS_EACH_ITERATION__

            b += r*jacob[i].transpose()*weight;

            nPtVisible++;
        }
//        else //if the point is out of scene, penalize this
//        {
////            r = 255.0;
////            b += r*jacob[i].transpose();
//            b += 255.0*jacob[i].transpose();
//        }

        cumm_r += (r*r);


    }
    b = -b;

    ROS_DEBUG( "# Points (of ref-frame) visible in now-frame : %d", nPtVisible );
    ROS_WARN_COND( nPtVisible < 50, "Too few pts visible. Signal end of iterations" );


    ROS_INFO( "Cummulative squared residue : %f", cumm_r );

    return (  (float)nPtVisible /  (float)pts3d_un_normalized.cols() );
//    if( nPtVisible < 50 )
//        return false;
//    else
//        return true;


}


/// Evals the weight as described in Cremers DVO paper
/// w(r) = 6/( 5 + (r/sigma)^2 )
/// where, sigma^2 = 1/n \sum_i r^2 6/ ( 5 + r/sigma )^2 ....iterative
float SolveDVO::getWeightOf( float r)
{
    return 6.0 / (5.0 + r*r/9 );
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
    ROS_DEBUG_STREAM( "wx : \n["<< wx << "]" );


    float theta = w.norm();
    ROS_DEBUG( "THETA = %lf", theta );
    if( theta < 1E-12 )
    {
        //outTr = Eigen::Matrix4d::Identity();
        ROS_WARN( "[exponentialMap] Theta too small returning identity");
        outR = Eigen::Matrix3f::Identity();
        outT = t;
        return;
    }

    Eigen::Matrix3f I3 = Eigen::Matrix3f::Identity();
    Eigen::RowVector3f zer_row = Eigen::RowVector3f::Zero();


    Eigen::Matrix3f xR = I3 + sin(theta)/theta * wx + (1.0-cos(theta))/(theta*theta) * wx * wx;

    Eigen::Matrix3f xV = I3 + (1-cos(theta))/(theta*theta) * wx + (theta-sin(theta))/(theta*theta*theta) * wx *wx;


    outT = xV*t;
    outR = xR;


//    outT = -xR.transpose() * outT;
//    outR.transposeInPlace();

    Eigen::Quaternionf quat(outR);
    ROS_INFO( "out Quaternion : %f %f %f %f", quat.x(), quat.y(), quat.z(), quat.w() );


    //outTr << xR, xV*t, zer_row, 1.0;
}

void SolveDVO::sOverlay( Eigen::MatrixXf eim, Eigen::MatrixXi mask, cv::Mat& xim, cv::Vec3b color )
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

int SolveDVO::countSelectedPts(Eigen::MatrixXf &Gx, Eigen::MatrixXf &Gy, Eigen::MatrixXi& roi)
{
    int count=0;
    assert( Gx.rows()>0 && Gy.rows() > 0 );
    assert( (Gx.rows() == Gy.rows())  &&  (Gx.cols() == Gy.cols()) );

    roi = Eigen::MatrixXi::Zero(Gx.rows(), Gx.cols());
    for( int xx=0 ; xx<Gx.cols() ; xx++ )
    {
        for( int yy=0 ; yy<Gx.rows() ; yy++ )
        {
            if(   GRAD_NORM( Gx(yy,xx), Gy(yy,xx) ) >  grad_thresh   )
            {
                count++;
                roi(yy,xx) = 1;
            }
        }
    }
    return count;
}

void SolveDVO::printRT(Eigen::Matrix3f& fR, Eigen::Vector3f& fT, const char * msg )
{
#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__
    ROS_INFO( "____%s____", msg );
    //ROS_INFO( "fR\n[" << fR << "]" );
    Eigen::Quaternionf quat(fR);
    ROS_INFO( "fR Quaternion : %f %f %f %f", quat.x(), quat.y(), quat.z(), quat.w() );
    ROS_INFO_STREAM( "fT : [" << fT.transpose() << "]" );
    ROS_INFO( "____" );
#endif
}


/// @brief Compute and visualize histogram of the residues
/// @param[in] residi : Residue at pixel location. Nx1
/// @note : This is customized to show integer valued bins only. 0 to 200 on x-axis. 0 to 2500 on y-axis
void SolveDVO::visualizeResidueHistogram(Eigen::VectorXf residi)
{
    // computation of histogram
    Eigen::VectorXf histogram = Eigen::VectorXf::Zero(260);
    for( int i=0 ; i<residi.rows() ; i++ )
    {
        histogram( (int)residi(i) + 1 )++;
        // +1 is to take care of the '-1' in the absolute-residues which means the residue at this point is not calculated
        // since the re-projected point is not visible in now frame
    }
    histogram /= residi.rows(); //normalized the histogram, now histogram \in [0,1]


    cv::Mat histPlot = cv::Mat::zeros( 500, 450, CV_8UC3 ) + cv::Scalar(255,255,255);
    //red-dots on vertical
    cv::line(histPlot, cv::Point(1,histPlot.rows-1), cv::Point(1,0), cv::Scalar(0,0,0) );
    for( float mag = 0 ; mag< .3f ; mag+=0.02f )
    {
        cv::circle(histPlot, cv::Point(1,histPlot.rows-20-int(mag*2000.0)),  2, cv::Scalar(0,0,255), -1);
        char toS[20];
        sprintf( toS, "%.2f", mag );
        cv::putText( histPlot, toS, cv::Point(10,histPlot.rows-20-int(mag*2000.0)), cv::FONT_HERSHEY_COMPLEX_SMALL, .7, cv::Scalar(0,0,0) );
    }


//    char tmpS[20];
//    sprintf( tmpS, "%f", histogram(i) );
//    cv::putText(histPlot, tmpS, cv::Point(250,20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0) );

    // histogram bars draw
    for(int i = 0; i < 256; i++)
    {
        float mag = histogram(i);
        cv::line(histPlot,cv::Point(2*i,histPlot.rows-20),cv::Point(2*i,histPlot.rows-20-int(mag*2000.0)),cv::Scalar(255,0,0));
        if( (2*(i-1))%50 == 0 )
        {
            cv::circle( histPlot, cv::Point(2*i,histPlot.rows-20), 2, cv::Scalar(0,0,255), -1 );
            char toS[20];
            sprintf( toS, "%d", i-1 );

            cv::putText( histPlot, toS,cv::Point(2*i,histPlot.rows-5), cv::FONT_HERSHEY_COMPLEX_SMALL, .7, cv::Scalar(0,0,0) );
        }
    }



    //plot laplacian distribution with [0, b_cap]
    float b_cap=0; //MLE of laplacian pdistriution parameters
    for( int i=0 ; i<residi.rows() ; i++ )
    {
        histogram( (int)residi(i) + 1 )++;
        b_cap += residi(i);
    }
    b_cap /= (residi.rows());

    char toS[100];
    sprintf( toS, "Laplacian Distribution MLE Estimate\nb_cap : %.3f", b_cap );
    cv::putText(histPlot, toS, cv::Point(40,20), cv::FONT_HERSHEY_COMPLEX_SMALL, .5, cv::Scalar(0,0,0) );
    for( int i=1 ; i<256 ; i++ )
    {
        float mag = 1/(2*b_cap) * exp( -(i-1)/b_cap );
        cv::circle(histPlot, cv::Point(2*i,histPlot.rows-20-mag*2000.), 2, cv::Scalar(255,255,0), -1 );
    }



    cv::imshow( "PDF of residues", histPlot );
}

void SolveDVO::visualizeResidueHeatMap(Eigen::MatrixXf eim, Eigen::MatrixXf residueAt)
{
    assert( (eim.rows() == residueAt.rows()) && "Image and mask rows must match");
    assert( (eim.cols() == residueAt.cols()) && "Image and mask cols must match");

    cv::Mat tmpIm, tmpIm8;
    cv::eigen2cv( eim, tmpIm );
    tmpIm.convertTo(tmpIm8, CV_8UC1);
    // make to 3-channel, ie. repeat gray in all 3 channels
    cv::Mat xim = cv::Mat::zeros(eim.rows(), eim.cols(), CV_8UC3 );
    std::vector<cv::Mat> ch;
    ch.push_back(tmpIm8);
    ch.push_back(tmpIm8);
    ch.push_back(tmpIm8);
    cv::merge(ch,xim);


    // make colors
    std::vector<cv::Vec3b> colors;
    colors.reserve(32);

// defining 32 colors
    {
        colors[0   ] = cv::Vec3b( 159     ,0     ,0 );
        colors[1   ] = cv::Vec3b( 191     ,0     ,0 );
        colors[2   ] = cv::Vec3b( 223     ,0     ,0 );
        colors[3   ] = cv::Vec3b( 255     ,0     ,0 );
        colors[4   ] = cv::Vec3b( 255    ,31     ,0 );
        colors[5   ] = cv::Vec3b( 255    ,63     ,0 );
        colors[6   ] = cv::Vec3b( 255    ,95     ,0 );
        colors[7   ] = cv::Vec3b( 255   ,127     ,0 );
        colors[8   ] = cv::Vec3b( 255   ,159     ,0 );
        colors[9   ] = cv::Vec3b( 255   ,191     ,0 );
        colors[10   ] = cv::Vec3b( 255   ,223     ,0 );
        colors[11   ] = cv::Vec3b( 255   ,255     ,0 );
        colors[12   ] = cv::Vec3b( 223   ,255    ,31 );
        colors[13   ] = cv::Vec3b( 191   ,255    ,63 );
        colors[14   ] = cv::Vec3b( 159   ,255    ,95 );
        colors[15   ] = cv::Vec3b( 127   ,255   ,127 );
        colors[16    ] = cv::Vec3b( 95   ,255   ,159 );
        colors[17    ] = cv::Vec3b( 63   ,255   ,191 );
        colors[18    ] = cv::Vec3b( 31   ,255   ,223 );
        colors[19     ] = cv::Vec3b( 0   ,255   ,255 );
        colors[20     ] = cv::Vec3b( 0   ,223   ,255 );
        colors[21     ] = cv::Vec3b( 0   ,191   ,255 );
        colors[22     ] = cv::Vec3b( 0   ,159   ,255 );
        colors[23     ] = cv::Vec3b( 0   ,127   ,255 );
        colors[24     ] = cv::Vec3b( 0    ,95   ,255 );
        colors[25     ] = cv::Vec3b( 0    ,63   ,255 );
        colors[26     ] = cv::Vec3b( 0    ,31   ,255 );
        colors[27     ] = cv::Vec3b( 0     ,0   ,255 );
        colors[28     ] = cv::Vec3b( 0     ,0   ,223 );
        colors[29     ] = cv::Vec3b( 0     ,0   ,191 );
        colors[30     ] = cv::Vec3b( 0     ,0   ,159 );
        colors[31     ] = cv::Vec3b( 0     ,0   ,127 );
    }

    for( int j=0 ; j<residueAt.cols() ; j++ )
    {
        for( int i=0 ; i<residueAt.rows() ; i++ )
        {
            float mag = residueAt(i,j);
            if( mag< 0.0f )
                continue;
            if( mag > 25.0f )
                xim.at<cv::Vec3b>(i,j) = colors[31];
            else
            {
                int colorIndx = (int)mag;
                xim.at<cv::Vec3b>(i,j) = colors[colorIndx];
            }
        }
    }

    cv::imshow( "residues heatmap", xim );

}

void SolveDVO::visualizeReprojectedDepth(Eigen::MatrixXf eim, Eigen::MatrixXf reprojDepth)
{
    assert( (eim.rows() == reprojDepth.rows()) && "Image and mask rows must match");
    assert( (eim.cols() == reprojDepth.cols()) && "Image and mask cols must match");

    cv::Mat tmpIm, tmpIm8;
    cv::eigen2cv( eim, tmpIm );
    tmpIm.convertTo(tmpIm8, CV_8UC1);
    // make to 3-channel, ie. repeat gray in all 3 channels
    cv::Mat xim = cv::Mat::zeros(eim.rows(), eim.cols(), CV_8UC3 );
    std::vector<cv::Mat> ch;
    ch.push_back(tmpIm8);
    ch.push_back(tmpIm8);
    ch.push_back(tmpIm8);
    cv::merge(ch,xim);


    std::vector<cv::Vec3b> colors;
    colors.reserve(64);
 // defining 64 colors
    {
        colors[0   ] = cv::Vec3b( 143     ,0     ,0 );
        colors[1   ] = cv::Vec3b( 159     ,0     ,0 );
        colors[2   ] = cv::Vec3b( 175     ,0     ,0 );
        colors[3   ] = cv::Vec3b( 191     ,0     ,0 );
        colors[4   ] = cv::Vec3b( 207     ,0     ,0 );
        colors[5   ] = cv::Vec3b( 223     ,0     ,0 );
        colors[6   ] = cv::Vec3b( 239     ,0     ,0 );
        colors[7   ] = cv::Vec3b( 255     ,0     ,0 );
        colors[8   ] = cv::Vec3b( 255    ,15     ,0 );
        colors[9   ] = cv::Vec3b( 255    ,31     ,0 );
       colors[10   ] = cv::Vec3b( 255    ,47     ,0 );
       colors[11   ] = cv::Vec3b( 255    ,63     ,0 );
       colors[12   ] = cv::Vec3b( 255    ,79     ,0 );
       colors[13   ] = cv::Vec3b( 255    ,95     ,0 );
       colors[14   ] = cv::Vec3b( 255   ,111     ,0 );
       colors[15   ] = cv::Vec3b( 255   ,127     ,0 );
       colors[16   ] = cv::Vec3b( 255   ,143     ,0 );
       colors[17   ] = cv::Vec3b( 255   ,159     ,0 );
       colors[18   ] = cv::Vec3b( 255   ,175     ,0 );
       colors[19   ] = cv::Vec3b( 255   ,191     ,0 );
       colors[20   ] = cv::Vec3b( 255   ,207     ,0 );
       colors[21   ] = cv::Vec3b( 255   ,223     ,0 );
       colors[22   ] = cv::Vec3b( 255   ,239     ,0 );
       colors[23   ] = cv::Vec3b( 255   ,255     ,0 );
       colors[24   ] = cv::Vec3b( 239   ,255    ,15 );
       colors[25   ] = cv::Vec3b( 223   ,255    ,31 );
       colors[26   ] = cv::Vec3b( 207   ,255    ,47 );
       colors[27   ] = cv::Vec3b( 191   ,255    ,63 );
       colors[28   ] = cv::Vec3b( 175   ,255    ,79 );
       colors[29   ] = cv::Vec3b( 159   ,255    ,95 );
       colors[30   ] = cv::Vec3b( 143   ,255   ,111 );
       colors[31   ] = cv::Vec3b( 127   ,255   ,127 );
       colors[32   ] = cv::Vec3b( 111   ,255   ,143 );
       colors[33    ] = cv::Vec3b( 95   ,255   ,159 );
       colors[34    ] = cv::Vec3b( 79   ,255   ,175 );
       colors[35    ] = cv::Vec3b( 63   ,255   ,191 );
       colors[36    ] = cv::Vec3b( 47   ,255   ,207 );
       colors[37    ] = cv::Vec3b( 31   ,255   ,223 );
       colors[38    ] = cv::Vec3b( 15   ,255   ,239 );
       colors[39     ] = cv::Vec3b( 0   ,255   ,255 );
       colors[40     ] = cv::Vec3b( 0   ,239   ,255 );
       colors[41     ] = cv::Vec3b( 0   ,223   ,255 );
       colors[42     ] = cv::Vec3b( 0   ,207   ,255 );
       colors[43     ] = cv::Vec3b( 0   ,191   ,255 );
       colors[44     ] = cv::Vec3b( 0   ,175   ,255 );
       colors[45     ] = cv::Vec3b( 0   ,159   ,255 );
       colors[46     ] = cv::Vec3b( 0   ,143   ,255 );
       colors[47     ] = cv::Vec3b( 0   ,127   ,255 );
       colors[48     ] = cv::Vec3b( 0   ,111   ,255 );
       colors[49     ] = cv::Vec3b( 0    ,95   ,255 );
       colors[50     ] = cv::Vec3b( 0    ,79   ,255 );
       colors[51     ] = cv::Vec3b( 0    ,63   ,255 );
       colors[52     ] = cv::Vec3b( 0    ,47   ,255 );
       colors[53     ] = cv::Vec3b( 0    ,31   ,255 );
       colors[54     ] = cv::Vec3b( 0    ,15   ,255 );
       colors[55     ] = cv::Vec3b( 0     ,0   ,255 );
       colors[56     ] = cv::Vec3b( 0     ,0   ,239 );
       colors[57     ] = cv::Vec3b( 0     ,0   ,223 );
       colors[58     ] = cv::Vec3b( 0     ,0   ,207 );
       colors[59     ] = cv::Vec3b( 0     ,0   ,191 );
       colors[60     ] = cv::Vec3b( 0     ,0   ,175 );
       colors[61     ] = cv::Vec3b( 0     ,0   ,159 );
       colors[62     ] = cv::Vec3b( 0     ,0   ,143 );
       colors[63     ] = cv::Vec3b( 0     ,0   ,127 );
    }

    for( int j=0 ; j<reprojDepth.cols() ; j++ )
    {
        for( int i=0 ; i<reprojDepth.rows() ; i++ )
        {
            float mag = reprojDepth(i,j); //mag \in (0, 10000)
            if( mag < 0.0f )
                continue;
            float colorIndx;

            if( mag < 500.0f )
                colorIndx = 0;
            else if( mag > 5500.0f )
                colorIndx = 0;
            else
                colorIndx = (int)((mag-500.0)/5000.0f * 60.0f) + 1;

            xim.at<cv::Vec3b>(i,j) = colors[colorIndx];
        }
    }

    cv::imshow( "reprojected depth heatmaps", xim );



}


/// @brief The event loop. Basically an ever running while with ros::spinOnce()
/// This is a re-implementation taking into care the memory scopes and also processing only points with higher image gradients
void SolveDVO::loop()
{
/*
    ros::Rate rate(30);
    long nFrame=0;
    // Pose of now frame wrt currently set reference frame
    Eigen::Matrix3f cR = Eigen::Matrix3f::Identity();
    Eigen::Vector3f cT = Eigen::Vector3f::Zero();

    // Pose of currently set Reference-frame (key frame)
    Eigen::Matrix3f keyR = Eigen::Matrix3f::Identity();
    Eigen::Vector3f keyT = Eigen::Vector3f::Zero();

    // Pose of now frame in global frame of reference
    Eigen::Matrix3f nR = Eigen::Matrix3f::Identity();
    Eigen::Vector3f nT = Eigen::Vector3f::Zero();

    while( ros::ok() )
    {
        ros::spinOnce();
        if( !(this->isFrameAvailable) )
            continue;

        ROS_INFO( "=-=-=-=-=-=-== Frame # %ld ==-=-=-=-=-=-=", nFrame );


        //if( (nFrame % 9000000) == 0 )
        if( signalGetNewRefImage == true )
        {
            setRefFrame();
            computeJacobian();
            keyR = nR;
            keyT = nT;
            cR = Eigen::Matrix3f::Identity();
            cT = Eigen::Vector3f::Zero();
            signalGetNewRefImage = false;

            publishReferencePointCloud(1);
            //publishPointCloud(_spCord[1], _intensities[1]);
        }


        setNowFrame();
        gaussNewtonIterations(3, 5, cR, cT );
        gaussNewtonIterations(2, 5, cR, cT );
        gaussNewtonIterations(1, 5, cR, cT );
        gaussNewtonIterations(0, 5, cR, cT );



        nT = keyT + keyR*cT;
        nR = keyR*cR;

        //publishBowl();
        publishPoseFinal(nR, nT);
        publishPoseWrtRef(cR, cT);
//        publishCurrentPointCloud(2);





        //Debugging display of re-projected points
        {
            int i = 0;
            //for( i=0 ; i<im_n.size() ; i++ )
//            {
//                char nowName[100], refName[100];
//                sprintf( nowName, "imNow%d", i );
//                sprintf( refName, "imRef%d", i );
//                imshowEigenImage(nowName, this->im_n[i]);
//                imshowEigenImage(refName, this->im_r[i]);
//            }
        cv::Mat outImg;
        sOverlay(im_n[__REPROJECTION_LEVEL], __now_roi_reproj, outImg, cv::Vec3b(0,255,0) );
        cv::imshow( "reprojected markers onto now-frame", outImg );
        cv::Mat outImg2;
        sOverlay(im_r[__REPROJECTION_LEVEL], _roi[__REPROJECTION_LEVEL], outImg2, cv::Vec3b(0,0,255) );
        cv::imshow( "markers on the ref-frame", outImg2);
        cv::moveWindow("reprojected markers onto now-drame", 0, 400 );
        cv::waitKey(3);
        }

        rate.sleep();
        nFrame++;
    }

*/




//    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
//        ros::console::notifyLoggerLevelsChanged();

    char frameFileName[50];
    const char * folder = "xdump_right2left";
    int iFrameNum = 0;
    const int END = 197;


    Eigen::Matrix3f cR = Eigen::Matrix3f::Identity();
    Eigen::Vector3f cT = Eigen::Vector3f::Zero();



    for( iFrameNum=iFrameNum+1 ; iFrameNum < END ; iFrameNum++ )
    {

        sprintf( frameFileName, "%s/framemono_%04d.xml", folder, iFrameNum );
        loadFromFile(frameFileName );


        if( signalGetNewRefImage == true )
        {
            setRefFrame();

            cR = Eigen::Matrix3f::Identity();
            cT = Eigen::Vector3f::Zero();

            computeJacobian();

            signalGetNewRefImage = false;

        }
        //else
        //{
            setNowFrame();

            gaussNewtonIterations(3, 7, cR, cT );
            gaussNewtonIterations(2, 7, cR, cT );
            gaussNewtonIterations(1, 7, cR, cT );
            gaussNewtonIterations(0, 100, cR, cT );
        //}

        // Some displaying
        {
            cv::Mat outImg;
            sOverlay(im_n[__REPROJECTION_LEVEL], __now_roi_reproj, outImg, cv::Vec3b(0,255,0) );
            cv::imshow( "reprojected markers onto now-frame", outImg );
            cv::Mat outImg2;
            sOverlay(im_r[__REPROJECTION_LEVEL], _roi[__REPROJECTION_LEVEL], outImg2, cv::Vec3b(0,0,255) );
            cv::imshow( "markers on the ref-frame", outImg2);
            cv::moveWindow("reprojected markers onto now-drame", 0, 400 );
            char ket = cv::waitKey(0);
            if( ket == 27 ) //if ESC Quit the program
                break;
            else if( ket == 'c' ) //if c pressed, then signal change of ref frame
                signalGetNewRefImage = true;
            else
                ROS_INFO( "/./././ Retrive Next Frame ././././ ");

        }




        //ros::Rate rate(30);
        //while( ros::ok() )
        {
            ros::spinOnce();
            //publishCurrentPointCloud();
            publishPointCloud( _spCord[0], _intensities[0] );

            publishPoseFinal(cR, cT);

            //    cv::waitKey(3);
            //    rate.sleep();
        }
    }




}




void SolveDVO::publishBowl()
{
    sensor_msgs::PointCloud pcl_msg;
    pcl_msg.header.frame_id = rviz_frame_id;
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


void SolveDVO::publishCurrentPointCloud( int level )
{
    assert( isCameraIntrinsicsAvailable && isNowFrameAvailable );
    assert( im_n.size() > 1 );


    sensor_msgs::PointCloud pcl_msg;
    pcl_msg.header.frame_id = rviz_frame_id;
    pcl_msg.header.stamp = ros::Time::now();


    float scaleFac = (float)pow(2,-level);
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
            float X = Z * (xx-scaleFac*cx) / (scaleFac*fx);
            float Y = Z * (yy-scaleFac*cy) / (scaleFac*fy);

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


void SolveDVO::publishReferencePointCloud( int level )
{
    assert( isCameraIntrinsicsAvailable && isRefFrameAvailable );
    assert( im_n.size() > 1 );


    sensor_msgs::PointCloud pcl_msg;
    pcl_msg.header.frame_id = rviz_frame_id;
    pcl_msg.header.stamp = ros::Time::now();


    float scaleFac = (float)pow(2,-level);
    Eigen::MatrixXf& _im = this->im_r[level];
    Eigen::MatrixXf& _dim = this->dim_r[level];

    sensor_msgs::ChannelFloat32 shade;
    shade.name = "intensity";


    for( int yy=0 ; yy<_im.rows() ; yy++ )
    {
        for( int xx=0 ; xx<_im.cols() ; xx++ )
        {
            float Z = _dim(yy,xx);
            if( Z < 10 )
                continue;
            float X = Z * (xx-scaleFac*cx) / (scaleFac*fx);
            float Y = Z * (yy-scaleFac*cy) / (scaleFac*fy);

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

void SolveDVO::publishPointCloud(SpaceCordList &spc, IntensityList& grays )
{
    assert( (spc.cols()>0)  &&  (spc.cols() == grays.rows())  && spc.rows()==3 );
    sensor_msgs::PointCloud pcl_msg;
    pcl_msg.header.frame_id = rviz_frame_id;
    pcl_msg.header.stamp = ros::Time::now();

    sensor_msgs::ChannelFloat32 shade;
    shade.name = "intensity";

    for( int i=0 ; i<grays.rows() ; i++ )
    {
        geometry_msgs::Point32 pt;
        pt.x = spc(0,i);
        pt.y = spc(1,i);
        pt.z = spc(2,i);

        pcl_msg.points.push_back(pt);
        shade.values.push_back( grays(i) );

    }
    pcl_msg.channels.push_back(shade);
    pub_pc.publish( pcl_msg );

}

/// @brief Publishes the pose of kth frame with respect to global frame (ie. 1st frame)
/// Publishes to var `pub_final_pose`
void SolveDVO::publishPoseFinal(Eigen::MatrixXf rot, Eigen::VectorXf tran)
{
    geometry_msgs::Pose rospose;
    matrixToPose(rot, tran, rospose);

    geometry_msgs::PoseStamped poseS;
    poseS.header.frame_id = rviz_frame_id;
    poseS.header.stamp = ros::Time::now();
    poseS.pose = rospose;

    pub_final_pose.publish( poseS );

}


/// @brief Publishes the pose of kth frame with respect to its ref frame
/// Publishes to var `pub_final_pose`
void SolveDVO::publishPoseWrtRef(Eigen::MatrixXf rot, Eigen::VectorXf tran)
{
    geometry_msgs::Pose rospose;
    matrixToPose(rot, tran, rospose);
    rospose.position.x += 500.0;

    geometry_msgs::PoseStamped poseS;
    poseS.header.frame_id = rviz_frame_id;
    poseS.header.stamp = ros::Time::now();
    poseS.pose = rospose;

    pub_pose_wrt_ref.publish( poseS );
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







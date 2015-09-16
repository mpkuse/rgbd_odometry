#include <SolveDVO.h>


/// default constructor. Init the subscribers
SolveDVO::SolveDVO()
{

    isCameraIntrinsicsAvailable = false;
    isFrameAvailable = false; isPrevFrameAvailable = false;
    isRefFrameAvailable = false;
    isNowFrameAvailable = false;
    isJacobianComputed = false;
    isRefDistTransfrmAvailable = false;

    signalGetNewRefImage = true;
    sprintf( signalGetNewRefImageMsg, "NO MSG");


    //
    // Setting up some global constants
    grad_thresh = 10; //this is currently not used
    ratio_of_visible_pts_thresh = 0.8;
    laplacianThreshExitCond = 3.0f;
    psiNormTerminationThreshold = 1.0E-7;
    trustRegionHyperSphereRadius = 0.003;



    // Iterations Config
    iterationsConfig.push_back(50); //level 0
    iterationsConfig.push_back(50); //level 1
    iterationsConfig.push_back(50); //level 2
    iterationsConfig.push_back(50); //level 3



    //
    // Setting up Publishers & Subscribers

    //sub = nh.subscribe( "odometry/rgbd", 2, &SolveDVO::imageArrivedCallBack, this );
    sub = nh.subscribe( "Xtion/rgbdPyramid", 10, &SolveDVO::imageArrivedCallBack, this );

  

    //
    // Visualization Class
    mviz.setNodeHandle(this); //this call also setups the publisher
    mviz.setRVizFrameID("denseVO");


    //
    // Open files for writing final poses (wrt to 1st frame)
#ifdef __WRITE_EST_POSE_TO_FILE
    estPoseFile.open(__WRITE_EST_POSE_TO_FILE, std::ios_base::trunc );
    if( estPoseFile.is_open() )
        ROS_INFO( "File opening success : %s", __WRITE_EST_POSE_TO_FILE);
    else
        ROS_ERROR( "File opening fail : %s", __WRITE_EST_POSE_TO_FILE );
#endif


#ifdef __WRITE_GT__POSE_TO_FILE
    gtPoseFile.open(__WRITE_GT__POSE_TO_FILE, std::ios_base::trunc );
    if( gtPoseFile.is_open() )
        ROS_INFO( "File opening success : %s", __WRITE_GT__POSE_TO_FILE);
    else
        ROS_ERROR( "File opening fail : %s", __WRITE_GT__POSE_TO_FILE );
#endif

}

SolveDVO::~SolveDVO()
{
#ifdef __WRITE_EST_POSE_TO_FILE
    estPoseFile.close();
#endif

#ifdef __WRITE_GT__POSE_TO_FILE
    gtPoseFile.close();
#endif
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
        ROS_ERROR_STREAM( "[SolveDVO::setCameraMatrix] Error opening camera "
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

    ROS_INFO( "[SolveDVO::setCameraMatrix] Camera Matrix & Distortion Coif Loaded");
    ROS_INFO( "Params File : %s", calibFile );
    ROS_INFO( "fx=%.4f, fy=%.4f, cx=%.4f, cy=%.4f", fx, fy, cx, cy );


    isCameraIntrinsicsAvailable = true;

}




/// @brief Display an Eigen::MatrixXd as an image (using opencv imshow())
void SolveDVO::imshowEigenImage(const char *winName, Eigen::MatrixXd& eim)
{
    cv::Mat tmp, tmpuint;
    cv::eigen2cv( eim, tmp );
    tmp.convertTo(tmpuint, CV_8UC1);

    cv::imshow(winName, tmpuint );
}

/// @brief Display an Eigen::MatrixXf as an image (using opencv imshow())
void SolveDVO::imshowEigenImage(const char *winName, Eigen::MatrixXf& eim)
{
    cv::Mat tmp, tmpuint;
    cv::eigen2cv( eim, tmp );
    tmp.convertTo(tmpuint, CV_8UC1);

    cv::imshow(winName, tmpuint );
}

/// @brief Load RGBD frame-data (stored) from file. Stored in OpenCV XML
/// @param[in] xmlFileName : OpenCV XML file to open
/// @returns false if error loading file. True on success
bool SolveDVO::loadFromFile(const char *xmlFileName)
{
    cv::FileStorage fs( xmlFileName, cv::FileStorage::READ );
    ROS_INFO_STREAM( "Loading : "<< xmlFileName );
    if( fs.isOpened() == false )
    {
        ROS_ERROR( "Cannot Open File %s", xmlFileName );
        return false;
    }

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

    return true;
}



std::string SolveDVO::cvMatType2str(int type)
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

/// @brief Given a mask of reference edge points, lists the edge points (as 3xN & 2xN)
/// @param[in] level : Pyramidal level
/// @param[in] refEdgePtsMask : Edge masks of the reference frame
/// @param[out] _3d : 3d edge points `E_i`s
/// @param[out] _2d : 2d edge points `e_i`s
/// @note Uses K (intrinsic parameters), and depth map of reference frame
void SolveDVO::enlistRefEdgePts(int level, Eigen::MatrixXi& refEdgePtsMask, SpaceCordList &_3d, ImCordList &_2d)
{
    assert( refEdgePtsMask.sum() == _3d.cols() );

    Eigen::MatrixXf _refDepth = dim_r[level];

    int nC=0;
    float scaleFac = (float)pow(2,-level);
    float tmpfx = 1./(scaleFac*fx);
    float tmpfy = 1./(scaleFac*fy);
    float tmpcx = scaleFac*cx;
    float tmpcy = scaleFac*cy;

    for( int xx=0 ; xx<refEdgePtsMask.cols() ; xx++ )
    {
        for( int yy=0 ; yy<refEdgePtsMask.rows() ; yy++ )
        {
            if( refEdgePtsMask(yy,xx) > 0 )
            {
                // _2d <- cummulate (yy,xx)
                _2d(0,nC) = xx;
                _2d(1,nC) = yy;

                // get 3d pt of yy,xx using the _depth (ref depth)
                float Z = _refDepth(yy,xx) / 1000.0f; //converting mm to m for numerical conditioning
                float X = Z * (xx-tmpcx) * tmpfx;
                float Y = Z * (yy-tmpcy) * tmpfy;


                // _3d <- cummulate 3dOf(yy,xx)
                _3d(0,nC) = X;
                _3d(1,nC) = Y;
                _3d(2,nC) = Z;

                nC++;
            }
        }
    }


}



/// Selects edge points of reference frame at each pyramidal level
void SolveDVO::preProcessRefFrame()
{
    assert( isRefFrameAvailable );
    _ref_edge_3d.clear();
    _ref_edge_2d.clear();
    _ref_roi_mask.clear();

    for( int level=0 ; level<im_r.size() ; level++ )
    {
        //
        /// Select points in Ref frame
        Eigen::MatrixXi _roi_ref;
        int nSelectedPts = selectedPts( level, _roi_ref );
        assert( nSelectedPts > 0 );


        //
        /// Enlist these selected pts (edge pts in ref) as 3d points using the depth-image obtained from Xtion
        SpaceCordList _3d = Eigen::MatrixXf::Zero(3, nSelectedPts);
        ImCordList _2d = Eigen::MatrixXf::Zero(2, nSelectedPts);
        enlistRefEdgePts( level, _roi_ref, _3d, _2d );


        //
        /// Push of pyramial-vector
        _ref_edge_3d.push_back(_3d);
        _ref_edge_2d.push_back(_2d);
        _ref_roi_mask.push_back(_roi_ref);


    }



}

/// Computes Jacobian of now frame around cR, cT
void SolveDVO::computeJacobianOfNowFrame(int level, Eigen::Matrix3f &cR, Eigen::Vector3f &cT, JacobianLongMatrix &Jcbian, Eigen::MatrixXf& reprojections)
{
    /// J_{at a pixel} = [ 2x1 grad of now distance ] x [ 2x3 projection jacobian ] x [ del_p / del_psi ]

    Eigen::MatrixXf _nowDist = now_distance_transform[level];


    /// "Step 1" : Gradient of dist transform image <br/>
    //    Eigen::MatrixXf dGx, dGy; //image gradient
    //    imageGradient(_nowDist, dGx, dGy);
    Eigen::MatrixXf dGx = now_DT_gradientX[level];
    Eigen::MatrixXf dGy = now_DT_gradientY[level];


    SpaceCordList _3d = _ref_edge_3d[level];
    ImCordList _2d = _ref_edge_2d[level];

    assert( _3d.rows()==3 && _2d.rows() == 2);
    assert( _3d.cols() == _2d.cols() );


    /// "Step 2" : Transform 3d points to ref frame denoted by cR, cT <br/>
    Eigen::MatrixXf cTRep;
    igl::repmat(cT,1,_3d.cols(),cTRep); // repeat cT col-wise (uses IGL library)
    SpaceCordList _3d_transformed = cR.transpose() * ( _3d - cTRep );

    /// "Step 3" : Corresponding reprojected pts. Project the transformed 3d points onto the image.
    /// This is done by de-homonegezing the equation (X/Z, Y/Z) followed by multiplication on K (camera intrinsic parameters)
    float scaleFac = (float)pow(2,-level);
    Eigen::Matrix3f scaleMatrix = Eigen::Matrix3f::Identity();
    scaleMatrix(0,0) = scaleFac;
    scaleMatrix(1,1) = scaleFac;

    Eigen::ArrayXXf lastRow_inv = _3d_transformed.row(2).array().inverse();
    for( int i=0 ; i<3 ; i++ )
        _3d_transformed.row(i).array() *= lastRow_inv;


    Eigen::MatrixXf _2d_reprojected = scaleMatrix * K * _3d_transformed;
    reprojections = _2d_reprojected;



    // declare variables
    Eigen::RowVector2f G;
    //Eigen::MatrixXf A1= Eigen::MatrixXf::Zero(2,3);
    Eigen::Matrix<float,2,3> A1;
    //Eigen::MatrixXf A2= Eigen::MatrixXf::Zero(3,6);
    Eigen::Matrix<float,3,6> A2;
    //Eigen::RowVectorXf J_i;
    Eigen::Matrix<float,6,1> J_i;
    Eigen::Matrix3f wx;
    Eigen::Vector3f tmp;

    //Sept 2015, Scaling trail
//    _3d_transformed /= 10.0f;


    /// "Step 4" : Loop over each 3d/2d edge pt of the (cR,cT) frame of reference (ie. ref Transformed) to compute jacobian at those pts
    int notJ = 0;
    const int nCols = _nowDist.cols();
    const int nRows = _nowDist.rows();

    for( int i=0 ; i<_3d.cols() ; i++ )
    {
        if( _2d_reprojected(0,i)<0 || _2d_reprojected(0,i)>nCols ||  _2d_reprojected(1,i)<0 || _2d_reprojected(1,i)>nRows) {
            notJ++;
            continue;
        }

        int xx = _2d_reprojected(0,i);
        int yy = _2d_reprojected(1,i);

        float X = _3d_transformed(0,i);
        float Y = _3d_transformed(1,i);
        float Z = _3d_transformed(2,i);

        // G
        G(0) = dGx(yy,xx);
        G(1) = dGy(yy,xx);

        // A1
        A1(0,0) = scaleFac*fx/Z;
        A1(0,1) = 0.;
        A1(0,2) = -scaleFac*fx*X/(Z*Z);
        A1(1,0) = 0.;
        A1(1,1) = scaleFac*fy/Z;
        A1(1,2) = -scaleFac*fy*Y/(Z*Z);

        // A2
        //A2 = Eigen::MatrixXf::Zero(3,6);
        A2.block<3,3>(0,0) = -cR.transpose() /** Eigen::MatrixXf::Identity(3,3)*/;

        tmp = cR.transpose() * _3d_transformed.col(i);

        to_se_3( tmp, wx );
        A2.block<3,3>(0,3) = wx;


        J_i = G * A1 * A2;
        Jcbian.block<1,6>(i,0) = J_i;
    }

#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__
    ROS_INFO( "Jacobians computed at %d of %d edge locations", _3d.cols() - notJ, _3d.cols() );
    ROS_INFO( "[computeJacobianOfNowFrame()] Visibility ratio : %2.2f", (float)(_3d.cols()-notJ) / (float)_3d.cols());
#endif

}

/// @brief Given the reprojection mask, get corresponding epsilons and also weight as a vector
/// Reads the epsilon (error) values at the reprojected location (not computed here). Errors-vals here are the
/// distance-transform values of the now frame at reprojected locations
/// @param[in] level : Pyramidal level [0-4]
/// @param[in] reprojections : 3xN matrix. Although last row is not usable
/// @param[out] epsilon : Error (not squared) at every reprojected pt.
/// @param[out] weights : corresponding weights (for solving equation with Huber norm)
/// @returns : Ratio of visible points to the total points tracked
/// \see computeJacobianOfNowFrame
float SolveDVO::getReprojectedEpsilons(int level, Eigen::MatrixXf& reprojections, Eigen::VectorXf& epsilon, Eigen::VectorXf &weights)
{
    Eigen::MatrixXf _nowDist = now_distance_transform[level];

    epsilon = Eigen::VectorXf::Zero( reprojections.cols() );
    weights = Eigen::VectorXf::Zero( reprojections.cols() );

    int notJ=0;
    for( int i=0 ; i<reprojections.cols() ; i++ )
    {
        if( reprojections(0,i)<0 || reprojections(0,i)>_nowDist.cols() ||  reprojections(1,i)<0 || reprojections(1,i)>_nowDist.rows()) {
            notJ++;
            //epsilon(i) = 200;
            //note : On having this one stange thing that happens is as points go out of scene the energy will increase even though the estimates
            // are getting better. Need a better way to take care of decreasing energy due to points going out of scene.
            continue;
        }

#ifdef __INTERPOLATE_DISTANCE_TRANSFORM
        epsilon(i) = interpolate(_nowDist,reprojections(1,i), reprojections(0,i) );
#else
        epsilon(i) = _nowDist( floor(reprojections(1,i)), floor(reprojections(0,i)) );
#endif


        weights(i) = getWeightOf( epsilon(i) );
    }
#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__
    ROS_INFO( "Epsilon computed at %d of %d reprojected points", reprojections.cols()-notJ,reprojections.cols());
    ROS_INFO( "[getReprojectedEpsilons()] Visibility ratio : %2.2f", (float)(reprojections.cols()-notJ) / (float)reprojections.cols());
#endif

    float ratio_of_visible_pts = (float)(reprojections.cols()-notJ) / (float)reprojections.cols();


    return ratio_of_visible_pts;

}


/// Given a list of co-ordinates make a mask of out of it
void SolveDVO::cordList_2_mask(Eigen::MatrixXf &list, Eigen::MatrixXi &mask)
{
    assert( mask.rows() > 0 && mask.cols() > 0 );

    for( int i=0 ; i<list.cols() ; i++ )
    {
        if( list(0,i)<0 || list(0,i)>mask.cols() ||  list(1,i)<0 || list(1,i)>mask.rows()) {
            continue;
        }

        int xx = list(0,i);
        int yy = list(1,i);

        mask(yy,xx) = 1;
    }

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
            dframe =  cv_bridge::toCvCopy(  msg->dframe[i] )->image ;

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
void SolveDVO::setRcvdFrameAsRefFrame()
{
    assert( isFrameAvailable );
    assert( rcvd_framemono.size() > 0  && rcvd_depth.size() > 0 );

    isRefFrameAvailable = false;
    im_r = rcvd_framemono;
    dim_r = rcvd_depth;
    isRefFrameAvailable = true;
    isJacobianComputed = false;


    computeDistTransfrmOfRef();
    // computeDistTransfromOfRef writes to following global variables
    //    ref_distance_transform
    //    ref_edge_map
    //    ref_DT_gradientX
    //    ref_DT_gradientY


}


/// @brief Sets the previous frame as reference frame. Uses `p_now_framemono` & `p_now_depth`
void SolveDVO::setPrevFrameAsRefFrame()
{
    assert( isPrevFrameAvailable );
    assert( rcvd_framemono.size() > 0  && rcvd_depth.size() > 0 );

    if( isPrevFrameAvailable == false )
        ROS_ERROR( "n-1 frame not available, can/must cause SIGSEGV" );

    isRefFrameAvailable = false;
    im_r = p_now_framemono;
    dim_r = p_now_depth;
    isRefFrameAvailable = true;
    isJacobianComputed = false;


    computeDistTransfrmOfRef();
    // computeDistTransfromOfRef writes to following global variables
    //    ref_distance_transform
    //    ref_edge_map
    //    ref_DT_gradientX
    //    ref_DT_gradientY


}


/// @brief Sets the rcvd frame as now frame
void SolveDVO::setRcvdFrameAsNowFrame()
{
    assert( isFrameAvailable && "FRAME NOT AVAILABLE" );
    assert( rcvd_framemono.size() > 0  && rcvd_depth.size() > 0 );

    //store current now frame as prev-now frame
    if( isNowFrameAvailable )
    {
        isPrevFrameAvailable = false;
        p_now_framemono = im_n;
        p_now_depth = dim_n;
        isPrevFrameAvailable = true;
    }

    isNowFrameAvailable = false;
    im_n = rcvd_framemono;
    dim_n = rcvd_depth;
    isNowFrameAvailable = true;

    computeDistTransfrmOfNow();
    // computeDistTransfrmOfNow writes to following variables
    //    now_distance_transform
    //    now_edge_map
    //    now_DT_gradientX
    //    now_DT_gradientY

}




void SolveDVO::runIterations(int level, int maxIterations, Eigen::Matrix3d& cR, Eigen::Vector3d& cT,
                                     Eigen::VectorXf& energyAtEachIteration, Eigen::VectorXf& finalEpsilons,
                                     Eigen::MatrixXf& finalReprojections, int& bestEnergyIndex, float& finalVisibleRatio)
{

    bool waitPerItr = true;
    assert( level >= 0 && level <= 3 );
    assert( maxIterations > 0 );
    assert( isRefFrameAvailable && isNowFrameAvailable && isCameraIntrinsicsAvailable );

#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__DISPLAY_ONLY
    Eigen::MatrixXf& _now = im_n[level];
    Eigen::MatrixXf& _nowDist = now_distance_transform[level];
#endif

    energyAtEachIteration = Eigen::VectorXf::Zero( maxIterations );

    ROS_DEBUG("-*-*-*-*- Start of Gauss-Newton Iterations (level=%d) -*-*-*-*-", level );

    ROS_DEBUG_STREAM( "init R :\n"<< cR );
    ROS_DEBUG_STREAM( "init T :\n"<< cT.transpose() );


    float prevTotalEpsilon = 1.0E10;

    float bestTotalEpsilon = 1.0E10;
    float bestRatioVisiblePts = 1.0f;
    Eigen::Matrix3d bestcR = Eigen::Matrix3d::Identity();
    Eigen::Vector3d bestcT = Eigen::Vector3d::Zero();
    int bestItrNumber = -1;
    Eigen::VectorXf bestEpsilon;
    Eigen::MatrixXf bestReprojections;

    double stepLength=1E-1;
    double BETA = 0.5;
    Eigen::VectorXd descentDirection = Eigen::VectorXd::Zero(6);
//    Eigen::VectorXd descentDirection = Eigen::VectorXd::Constant(6,1,1.0);
//    descentDirection /= descentDirection.norm();
    Eigen::VectorXd g_prev = Eigen::VectorXd::Zero(6);
    for( int itr=0 ; itr< maxIterations ; itr++ )
    {

#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__
        ROS_INFO( "== Iteration %d ==", itr );
#endif


        //
        // Compute Jacobian of the **NOW** frame (not the ref frame) (this is forward formulation)
        //          Be careful here since computeJacobian() function computes jacobiann at zero of the reference frame. Here we have to compute jacobian at
        //                    **R_cap**,    **T_cap**.
        SpaceCordList _refEdge = _ref_edge_3d[level];
        JacobianLongMatrix Jcbian = Eigen::MatrixXf::Zero(_refEdge.cols(), 6 );
        Eigen::MatrixXf reprojections;
        Eigen::Matrix3f cR_32 = cR.cast<float>();
        Eigen::Vector3f cT_32 = cT.cast<float>();
        computeJacobianOfNowFrame( level, cR_32, cT_32, Jcbian, reprojections );


#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__
        ROS_INFO( "size of reprojection : %d %d", reprojections.rows(), reprojections.cols() );
#endif


        //
        // Get corresponding epsilons
        //       Get distances at points given by `reprojections` 2xN matrix
        Eigen::VectorXf epsilon, weights;
        float ratio_of_visible_pts = getReprojectedEpsilons( level, reprojections, epsilon, weights );
//        float currentTotalEpsilon = epsilon.norm(); //epsilon.sum() ;
        float currentTotalEpsilon = aggregateEpsilons( epsilon );
        energyAtEachIteration[itr] = currentTotalEpsilon;
#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__
        ROS_INFO( "#%d : Total Epsilon : %f", itr, currentTotalEpsilon );
#endif

        // store the best epsilon
        if( currentTotalEpsilon <= bestTotalEpsilon )
        {
            bestTotalEpsilon = currentTotalEpsilon;
            bestRatioVisiblePts = ratio_of_visible_pts;
            bestcR = cR;
            bestcT = cT;
            bestItrNumber = itr;
            bestEpsilon = epsilon;
            bestReprojections = reprojections;
        }
#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__
        ROS_INFO( "Best iteration so far is #%d. Total epsilon : %f", bestItrNumber, bestTotalEpsilon );
#endif



        //
        // Weighting the Jacobians
        Eigen::MatrixXf JTW = Jcbian.transpose(); //J' * W
        for( int i=0 ; i<weights.rows() ; i++ )
            JTW.col(i) *= weights(i);


        Eigen::MatrixXd JTW_64 = JTW.cast<double>();
        Eigen::VectorXd epsilon_64 = epsilon.cast<double>();


        // Custom Pre-conditioner
        Eigen::Matrix<double,6,1> PVec;
        double PFactor = .5;
        double factorBig = 1.0 / ( 3*(1+PFactor) );
        double factorSmall = PFactor / ( 3*(1+PFactor) );
        //PVec<<factorBig, factorBig, factorBig   , factorSmall, factorSmall, factorSmall;
        PVec<< 1.0,1.0,1.0,    PFactor, PFactor, PFactor;
        Eigen::Matrix<double,6,6> P = PVec.asDiagonal();



#ifdef __ENABLE_L2_REGULARIZATION
        // L2 Regularization
        Sophus::SE3d cGrp;
        cGrp.setRotationMatrix( cR );
        cGrp.translation() = cT;
        Eigen::VectorXd cPsi = Sophus::SE3d::log(cGrp);
        if( cPsi.norm() > 0 )
            cPsi = cPsi / cPsi.norm();
        double regularizationLambda = 0.05;
#endif //__ENABLE_L2_REGULARIZATION



        //////////////////////////////////////////////////
        //                  STEP PARAMOUNT              //
        //      Input  : Jcbian & JTW, epsilon          //
        //      Output : psi                            //
        //////////////////////////////////////////////////
        /*
         // LM heuristic
        if( prevTotalEpsilon < currentTotalEpsilon ){
#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__
            ROS_INFO( "Registed INCREASE of energy at this iteration"); // divergence ... bad (so reduce step length )
#endif
            stepLength =(stepLength<1E-9)?stepLength: stepLength / 2.0f; //done to avoid floating point problems
            //BETA = 0.25;
        }
        else {
#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__
            ROS_INFO( "Registed DECREASE of energy at this iteration"); // convergence ... good (so increase step length)
#endif
            stepLength =(stepLength>1E9)?stepLength: stepLength * 2.0f; //done to avoid floating point problems
            //BETA = 0.75;
        }
        */



        // Square summable but not summable seq of step-length (as recommended in Boyd's documents)
        stepLength = 9.0 * 1.0E-2 / (  (itr>5)?(double)(itr-4):1.0  );  //dont turn down the rate too soon
//        BETA = 0.99 * (double)itr/(double)maxIterations;


        Eigen::VectorXd g = JTW_64 * epsilon_64; // subgradient of \Sum{ V^2(.) }
        double g_norm = g.norm();
//        g = g/g_norm;


        //Mike Shuster's Advice. Increase stepLen when grads point in same direction
        bool gradPointsInSameDirection=true;
        if( ((g.transpose() * g_prev) > 0)   ) { //same direction
//            stepLength =(stepLength>1000)?stepLength: stepLength * 1.2f; //done to avoid floating point problems
            gradPointsInSameDirection=true;
        }
        else {
//            stepLength =(stepLength<1E-9)?stepLength: stepLength / 2.0f; //done to avoid floating point problems
            gradPointsInSameDirection=false;
        }



#ifdef __ENABLE_L2_REGULARIZATION
        g += regularizationLambda * cPsi; //adding regularization term
#endif //__ENABLE_L2_REGULARIZATION

        descentDirection = (1.0f-BETA)*g + BETA*descentDirection;
//        descentDirection = (1.0f-BETA)*g + BETA*g_prev;
//        descentDirection = g;

        double s_norm = descentDirection.norm();
//        descentDirection = descentDirection / s_norm;
        g_prev = g;





        // Printing about gradients
#ifdef __PRINT_GRAD_DIRECTION_EACH_ITERATION
        char xcttt[500];
        sprintf( xcttt, "raw grad (%.4lf, %.4lf %.4lf %s dirn)", g_norm, s_norm, s_norm/g_norm, gradPointsInSameDirection?"same":"opposite" );
        printDescentDirection( g, xcttt );
#endif




        Eigen::VectorXd psi = - stepLength* P * descentDirection;



#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__
        ROS_INFO_STREAM( "psi : (raw) [ "<< psi.transpose() << "]\n"<< "|psi| : "<< psi.norm() );
#endif


        // Possible projection of `psi` on a hyper sphere (of radius \delta)
        //      Projected subgradient method / Trust region enforcement step
        double norm = psi.norm();
        float rot_norm = psi.head(3).norm();   // rotation norm
        float trans_norm = psi.tail(3).norm(); //translation norm
        if(norm > (double) trustRegionHyperSphereRadius ) //total component
        {
            psi = psi / norm * (double) trustRegionHyperSphereRadius;
//            ROS_INFO( "[P] Project on Trust Region" );
        }
        else
//            ROS_INFO( "[N] Already in Trust Region" );
//        if( rot_norm > 0.00001f ) //rotation component
//        {
//            psi.tail(3) = psi.tail(3) / rot_norm * 0.00001f;
//        }
//        if( trans_norm > 0.01f ) //translation component
//        {
//            psi.head(3) = psi.head(3) / trans_norm * 0.01f;
//        }



#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__
        ROS_INFO_STREAM( "psi : (after trust region constraint) [ "<< psi.transpose() << "]\n"<< "|psi| : "<< psi.norm() );
        ROS_INFO_STREAM( "descentDirection : [ "<< descentDirection.transpose() << "]" );
        ROS_INFO( "alpha_k : %f", stepLength );
        ROS_INFO_STREAM( "|psi| : "<< psi.norm() );
#endif


        //////////////////////////////////////////////////
        //                 END   PARAMOUNT              //
        //      Input  : Jcbian & JTW, epsilon          //
        //      Output : psi                            //
        //////////////////////////////////////////////////




        //
        // Early Termination ?
        if( psi.norm() < psiNormTerminationThreshold )
        {
#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__
            ROS_INFO( "Early Termination. psi.norm() {%f} smaller than %f", psi.norm(), psiNormTerminationThreshold );
#endif
            break;
        }

        //if( (itr - bestItrNumber) > 10 ) //if no betterment in function value for X iterations then give up
        //    break;




        // keeping track of epsilon frm previous step
        prevTotalEpsilon = currentTotalEpsilon;


        //
        // Update R_cap, T_cap
            //Eigen::Matrix3f wx;
            //to_se_3( psi(3), psi(4), psi(5), wx );
            //cR = cR * ( Eigen::Matrix3f::Identity() + wx );
            //cT = cT + Eigen::Vector3f(psi(0), psi(1), psi(2));






        Eigen::Matrix3d xRot = Eigen::Matrix3d::Identity();
        Eigen::Vector3d xTrans = Eigen::Vector3d::Zero();

        Sophus::SE3d mat = Sophus::SE3d::exp(psi);
        xRot = mat.rotationMatrix();
        xTrans = mat.translation();






//        cT = cR*xTrans + cT;
//        cR = cR * xRot;
        cT += cR*xTrans;
        cR *= xRot;
#ifdef __ENABLE_ROTATIONIZE__
        rotationize(cR);
#endif

#ifdef __PRINT_POSE_EACH_ITERATION
        char msgStr[100];
        sprintf( msgStr, "StepLen : %-.5f", stepLength );
        printRT( cR, cT, msgStr );
#endif



#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__DISPLAY_ONLY
        //
        // DISPLAY
        //printRT( cR, cT, "Updated cR,cT");
        if( waitPerItr )
        {
        if( __REPROJECTION_LEVEL == level ){
        Eigen::MatrixXi reprojectedMask = Eigen::MatrixXi::Zero(_now.rows(), _now.cols());
        cordList_2_mask(reprojections, reprojectedMask);

#ifndef __MINIMAL_DISPLAY
        cv::Mat outIm, outImGray, outRef;
        sOverlay(_nowDist, reprojectedMask, outIm, cv::Vec3b(0,255,0));
        sOverlay(_now, reprojectedMask, outImGray, cv::Vec3b(0,255,0));

        sOverlay(im_r[level], ref_edge_map[level], outRef, cv::Vec3b(0,0,255));
        cv::imshow( "reprojection with cR,cT (on now dist-tran", outIm);
        cv::imshow( "reprojection with cR,cT (on now gray", outImGray);
        cv::imshow( "selected edges on ref", outRef);
        processResidueHistogram( epsilon, false );

#endif

#ifdef __WRITE_EACH_ITERATION_IMAGE_TO_FILE
        char ffffName[500], fffNameDT[500];
        FRG34h_casualIterationCount = itr;
        sprintf( ffffName, "%d.png", FRG34h_casualIterationCount );
        sprintf( fffNameDT, "DT_%d.png", FRG34h_casualIterationCount );
        if( FRG34h_casualIterationCount%5 == 0 ) {
        cv::imwrite( ffffName, outImGray );
        cv::imwrite( fffNameDT, outIm );
        }

        ROS_INFO( "iteration image to file written");

#endif

        visualizeDistanceResidueHeatMap(im_n[level], reprojectedMask, now_distance_transform[level] );
        visualizeEnergyProgress( energyAtEachIteration, bestItrNumber, (energyAtEachIteration.rows() < 300)?4:1 );



#if  !defined(__PRINT_POSE_EACH_ITERATION)  &&  !defined(__PRINT_GRAD_DIRECTION_EACH_ITERATION)
        ROS_ERROR( "Waiting in runIterations (#%d)... ", itr );
#endif
        char ch = cv::waitKey(0);
        if( ch == 27 ){ // ESC
            ROS_ERROR( "ESC pressed quitting...");
            exit(1);
        }
        if( ch == 'b')
            break;
        if( ch == 'c') // do not wait after this
            waitPerItr = false;
        }

        }
        //
        // END DISPLAY
#endif


    }



    // Set : f^* = min{ f_best^(k-1), f^k }
    cR = bestcR;
#ifdef __ENABLE_ROTATIONIZE__
    rotationize(cR);
#endif
    cT = bestcT;
    finalEpsilons = bestEpsilon;
    finalReprojections = bestReprojections;
    bestEnergyIndex = bestItrNumber;
    finalVisibleRatio = bestRatioVisiblePts;


#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__DISPLAY_ONLY
    ROS_DEBUG( "Best Energy Achieved (%f) in iteration #%d with visibility-ratio of %2.2f", bestTotalEpsilon, bestItrNumber, finalVisibleRatio );
    ROS_DEBUG_STREAM( "final R :\n"<< cR );
    ROS_DEBUG_STREAM( "final T :\n"<< cT.transpose() );
    ROS_INFO("-*-*-*-*- End of Gauss-Newton Iterations (level=%d) -*-*-*-*- ", level );
#endif



}




/// @brief Updates the current estimates of R_{t}, T_{t} using R_h, T_h ie. the best deltas from previous iteration
void SolveDVO::updateEstimates(Eigen::Matrix3f &cR, Eigen::Vector3f &cT, Eigen::Matrix3f &R_h, Eigen::Vector3f &T_h)
{
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



/// Evals the weight as described in Cremers DVO paper
/// w(r) = 6/( 5 + (r/sigma)^2 )
/// where, sigma^2 = 1/n \sum_i r^2 6/ ( 5 + r/sigma )^2 ....iterative
float SolveDVO::getWeightOf( float r)
{
    //return 1.0f;
    //return 1.0f + r*r;
    return 6.0 / (6.0 + r*r/.25 );
    //return (float)std::exp( -fabs(r/9) );
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
//    cv::Mat kernX = (cv::Mat_<float>(3,3) <<  0, 0,  0,
//            0,  -1.0, 1.0,
//            0, 0,  0);
//    cv::Mat kernY = (cv::Mat_<float>(3,3) <<  0, 0,  0,
//            0,  -1.0, 0,
//            0, 1.0,  0);

    cv::Mat kernX = (cv::Mat_<float>(3,3) <<  0, 0,  0,
            -0.5,  0.0, .5,
            0, 0,  0);
    cv::Mat kernY = (cv::Mat_<float>(3,3) <<  0, -0.5,  0,
            0,  0.0, 0,
            0, 0.5,  0);




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

/// @brief Makes a map of selected point.
/// Selects the interest points based on if it is edge-point
int SolveDVO::selectedPts(int level, Eigen::MatrixXi& roi)
{
    assert( isRefDistTransfrmAvailable );
    Eigen::MatrixXi _refEdge = ref_edge_map[level];
    Eigen::MatrixXf _refDepth = dim_r[level];


    int count=0;

    assert( _refEdge.rows()>0 && _refEdge.rows() > 0 );
    assert( (_refEdge.rows() == _refEdge.rows())  &&  (_refEdge.cols() == _refEdge.cols()) );



    roi = Eigen::MatrixXi::Zero(_refEdge.rows(), _refEdge.cols());
    for( int xx=0 ; xx<_refEdge.cols() ; xx++ )
    {
        for( int yy=0 ; yy<_refEdge.rows() ; yy++ )
        {
            //if(   GRAD_NORM( Gx(yy,xx), Gy(yy,xx) ) >  grad_thresh   )
//            if( (_refEdge(yy,xx) > 0) && (_refDepth(yy,xx) > 100.0f)  &&  (_refDepth(yy,xx) < 5000.0f) )
            if( (_refEdge(yy,xx) > 0) && (_refDepth(yy,xx) > 100.0f) )
            {
                count++;
                roi(yy,xx) = 1;
            }
        }
    }


    int roiSum = roi.sum();
    assert( roiSum == count );

    return roiSum;
}

/// @brief Projects a 3x3 matrix into the orthogonal group.
/// The solution is to enforce singular values to be either +-1 in place og 0.999999.
/// @param [in,out] R : The matrix which will be converted to orthogonal matrix
void SolveDVO::rotationize(Eigen::Matrix3d &R)
{
    Eigen::JacobiSVD<Eigen::Matrix3d, Eigen::NoQRPreconditioner> svd(R, Eigen::ComputeFullU | Eigen:: ComputeFullV);

    Eigen::Vector3d SVec = svd.singularValues();
    Eigen::MatrixXd S = Eigen::MatrixXd::Identity(3,3);
    S(0,0) = (SVec(0)>0)?1.0:-1.0;
    S(1,1) = (SVec(1)>0)?1.0:-1.0;
    S(2,2) = (SVec(2)>0)?1.0:-1.0;


    R = svd.matrixU() * S * svd.matrixV().transpose();

}

/// @brief Gives the value of the matrix F at position (y,x). y:rowIndx. x:colIndx. Uses Bilinear interpolation
float SolveDVO::interpolate(Eigen::MatrixXf& F, float ry, float rx)
{
    //
    // Bilinear interpolation
//    ROS_INFO( "reprojected : %f, %f", ry, rx );
    int ry_d = (int)floor(ry);
    int rx_d = (int)floor(rx);
    int ry_u = (int)ceil(ry);
    int rx_u = (int)ceil(rx);
    float inc_x = rx - rx_d; //floating part in x
    float inc_y = ry - ry_d; //floating part in y

    // In the following d_xdyd_xuyd means it is a linear combination of 2 terms viz.
    // 1. first term from down of x (xd), down of y (yd)
    // 2. 2nd term from up of x (xu), down of y (yd)
    float f_xdyd_xuyd =  sqrt( (1.0f - inc_x) * F(ry_d,rx_d)* F(ry_d,rx_d) + (inc_x) * F(ry_d,rx_u)* F(ry_d,rx_u) );
    float f_xdyu_xuyu =  sqrt( (1.0f - inc_x) * F(ry_u,rx_d)* F(ry_u,rx_d) + (inc_x) * F(ry_u,rx_u) *F(ry_u,rx_u) );

    // Interpolate the above obtained intermediate terms
    float f_xy = sqrt( (1.0f - inc_y) * f_xdyd_xuyd*f_xdyd_xuyd + inc_y * f_xdyu_xuyu*f_xdyu_xuyu );

    return f_xy;

}

float SolveDVO::aggregateEpsilons(Eigen::VectorXf epsilon)
{
    return epsilon.norm();
    float sum=0;
    for( int p=0 ; p<epsilon.rows() ; p++ )
    {
        float e = epsilon(p);
        sum += getWeightOf(e) * e * e;
    }
    return sqrt(sum);
}

void SolveDVO::printRT(Eigen::Matrix3f& fR, Eigen::Vector3f& fT, const char * msg )
{
//#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__
  //  ROS_INFO( "____%s____", msg );
    //ROS_INFO( "fR\n[" << fR << "]" );
    Eigen::Quaternionf quat(fR);
    ROS_INFO( "6-DOF (%s) : %-.4f %-.4f %-.4f %-.4f : %-.4f %-.4f %-.4f", msg, quat.x(), quat.y(), quat.z(), quat.w(), fT(0), fT(1), fT(2) );
    //ROS_INFO_STREAM( "fT : [" << fT.transpose() << "]" );
    //ROS_INFO( "____" );
    //#endif
}

void SolveDVO::printRT(Eigen::Matrix3d& fR, Eigen::Vector3d& fT, const char * msg )
{
    Eigen::Quaterniond quat(fR);
    ROS_INFO( "6-DOF (%s) : %-.4lf %-.4lf %-.4lf %-.4lf : %-.4lf %-.4lf %-.4lf", msg, quat.x(), quat.y(), quat.z(), quat.w(), fT(0), fT(1), fT(2) );

}

void SolveDVO::printPose(geometry_msgs::Pose &rospose, const char *msg, std::ostream& stream )
{
    ROS_INFO( "%s : [ %.4lf %.4lf %.4lf %.4lf :: %.4lf %.4lf %.4lf ]", msg, rospose.orientation.x, rospose.orientation.y, rospose.orientation.z, rospose.orientation.w,
              rospose.position.x, rospose.position.y, rospose.position.z );

    if( stream != std::cout )
    {
        //        ROS_ERROR( "PRINTING TO FILE" );
        stream<< rospose.orientation.x<< " "<< rospose.orientation.y<< " "<< rospose.orientation.z<< " "<< rospose.orientation.w<< " "<<
                 rospose.position.x<< " "<< rospose.position.y<< " "<< rospose.position.z<< "\n";
        stream.flush();
    }

}

/// Prints the 6-vector representing direction of descent
void SolveDVO::printDescentDirection(Eigen::VectorXd d, const char *msg)
{
    assert( d.rows() == 6 );

    ROS_INFO( "Direction (%s) : %-4.4lf %-4.4lf %-4.4lf ::: %-4.4lf %-4.4lf %-4.4lf", msg, d(0), d(1), d(2),    d(3), d(4), d(5) );
}

void SolveDVO::printDescentDirection(Eigen::VectorXf d, const char *msg)
{
    assert( d.rows() == 6 );

    ROS_INFO( "Direction (%s) : %-4.4f %-4.4f %-4.4f ::: %-4.4f %-4.4f %-4.4f", msg, d(0), d(1), d(2),    d(3), d(4), d(5) );
}

float SolveDVO::getDriftFromPose(geometry_msgs::Pose &p1, geometry_msgs::Pose &p2)
{
    float dx = p1.position.x - p2.position.x;
    float dy = p1.position.y - p2.position.y;
    float dz = p1.position.z - p2.position.z;

    return (float)sqrt( dx*dx + dy*dy + dz*dz );
}

void SolveDVO::analyzeDriftVector(std::vector<float> &v)
{
    float mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    std::nth_element(v.begin(), v.begin() + v.size()/2, v.end());
    float median = v[v.size()/2];
    float rms = (float)sqrt(  std::inner_product(v.begin(), v.end(), v.begin(), 0.0) / v.size() );
    //ROS_INFO( "Mean : %2.4f :::: Median : %2.4f :::: RMS : %2.4f", mean, median, rms );

    float perSec = (float)v.size() / 30.0f;
    ROS_INFO( "Mean : %2.4f :::: Median : %2.4f :::: RMS : %2.4f", mean/perSec, median/perSec, rms/perSec );
}


/// @brief Compute and visualize histogram of the residues
/// @param[in] residi : Residue at pixel location. Nx1
/// @param[in] quite : `true` will suppress the display. By default the display is off
/// @returns : Return the `b` param of the MLE laplacian distribution
/// @note : This is customized to show integer valued bins only. 0 to 200 on x-axis. 0 to 2500 on y-axis
float SolveDVO::processResidueHistogram(Eigen::VectorXf& residi, bool quite=true)
{
    float toReturn_bCap=0.0f;

    // computation of histogram
    Eigen::VectorXf histogram = Eigen::VectorXf::Zero(260);
    for( int i=0 ; i<residi.rows() ; i++ )
    {
        histogram( (int)residi(i) + 1 )++;
        // +1 is to take care of the '-1' in the absolute-residues which means the residue at this point is not calculated
        // since the re-projected point is not visible in now frame
    }
    histogram /= residi.rows(); //normalized the histogram, now histogram \in [0,1]


    cv::Mat histPlot;
    if( quite == false )
    {
        histPlot = cv::Mat::zeros( 500, 450, CV_8UC3 ) + cv::Scalar(255,255,255);
        //red-dots on vertical
        cv::line(histPlot, cv::Point(1,histPlot.rows-1), cv::Point(1,0), cv::Scalar(0,0,0) );
        for( float mag = 0 ; mag< .95f ; mag+=0.05f )
        {
            cv::circle(histPlot, cv::Point(1,histPlot.rows-20-int(mag*500.0)),  2, cv::Scalar(0,0,255), -1);
            char toS[20];
            sprintf( toS, "%.2f", mag );
            cv::putText( histPlot, toS, cv::Point(10,histPlot.rows-20-int(mag*500.0)), cv::FONT_HERSHEY_COMPLEX_SMALL, .7, cv::Scalar(0,0,0) );
        }
    }


//    char tmpS[20];
//    sprintf( tmpS, "%f", histogram(i) );
//    cv::putText(histPlot, tmpS, cv::Point(250,20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0) );

    if( quite == false )
    {
        // histogram bars draw
        for(int i = 0; i < 256; i++)
        {
            float mag = histogram(i);
            cv::line(histPlot,cv::Point(2*i,histPlot.rows-20),cv::Point(2*i,histPlot.rows-20-int(mag*500.0)),cv::Scalar(255,0,0));
            if( (2*(i-1))%50 == 0 )
            {
                cv::circle( histPlot, cv::Point(2*i,histPlot.rows-20), 2, cv::Scalar(0,0,255), -1 );
                char toS[20];
                sprintf( toS, "%d", i-1 );

                cv::putText( histPlot, toS,cv::Point(2*i,histPlot.rows-5), cv::FONT_HERSHEY_COMPLEX_SMALL, .7, cv::Scalar(0,0,0) );
            }
        }
    }



    //MLE of laplacian distribution parameters
    float b_cap=0;
    for( int i=0 ; i<residi.rows() ; i++ )
    {
        histogram( (int)residi(i) + 1 )++;
        b_cap += residi(i);
    }
    b_cap /= (residi.rows());
    toReturn_bCap = b_cap;


    if( quite == false )
    {
        char toS[100];
        //plot laplacian distribution with [0, b_cap]
        sprintf( toS, "Laplacian Distribution MLE Estimate\nb_cap : %.3f", b_cap );
        cv::putText(histPlot, toS, cv::Point(40,20), cv::FONT_HERSHEY_COMPLEX_SMALL, .5, cv::Scalar(0,0,0) );
        // Plot the estimated laplacian distribution
        for( int i=1 ; i<256 ; i++ )
        {
            float mag = 1/(2*b_cap) * exp( -(i-1)/b_cap );
            cv::circle(histPlot, cv::Point(2*i,histPlot.rows-20-mag*500.), 2, cv::Scalar(255,255,0), -1 );
        }



        cv::imshow( "PDF of residues", histPlot );
    }

    return toReturn_bCap;
}

void SolveDVO::visualizeResidueHeatMap(Eigen::MatrixXf &eim, Eigen::MatrixXf &residueAt)
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
    FColorMap fcm(64);

    for( int j=0 ; j<residueAt.cols() ; j++ )
    {
        for( int i=0 ; i<residueAt.rows() ; i++ )
        {
            float mag = residueAt(i,j);

            if( mag< 0.0f )
                continue;
            if( mag < 2.0f )
                xim.at<cv::Vec3b>(i,j) = fcm.at(0); //colors[0];
            else
            {
                int colorIndx = (int)mag;
                xim.at<cv::Vec3b>(i,j) = fcm.at(colorIndx); //colors[colorIndx];
            }
        }
    }

    cv::imshow( "residues heatmap", xim );

}


void SolveDVO::visualizeDistanceResidueHeatMap(Eigen::MatrixXf &eim, Eigen::MatrixXi &reprojectionMask, Eigen::MatrixXf &nowDistMap )
{
    assert( (eim.rows() == reprojectionMask.rows()) && "Image and mask rows must match");
    assert( (eim.cols() == reprojectionMask.cols()) && "Image and mask cols must match");

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
    FColorMap fcm(64);

    for( int j=0 ; j<eim.cols() ; j++ )
    {
        for( int i=0 ; i<eim.rows() ; i++ )
        {
            float mag;
            if( reprojectionMask(i,j) > 0 )
                mag = nowDistMap(i,j);
            else
                mag = -1.0f;

            if( mag< 0.0f )
                continue;

            int colorIndx = 0;
            if( mag > 60 )
                colorIndx = 63;
            else
                colorIndx = (int)mag;


            xim.at<cv::Vec3b>(i,j) = fcm.at(colorIndx);

        }
    }

    cv::imshow( "residues heatmap", xim );

#ifdef __WRITE_EACH_ITERATION_IMAGE_TO_FILE
        char ffffName[500];
        sprintf( ffffName, "%d_heatmap.png", FRG34h_casualIterationCount );
        if( FRG34h_casualIterationCount%5 == 0 )
        cv::imwrite( ffffName, xim );
        ROS_INFO( "iteration image to file written");
#endif

}

void SolveDVO::visualizeEnergyProgress(Eigen::VectorXf energyAtEachIteration, int bestEnergyIndex, int XSPACING )
{
    assert( energyAtEachIteration.rows() > 0 );
    energyAtEachIteration *= 10.0;

    int len = energyAtEachIteration.rows();
    float min = energyAtEachIteration.minCoeff();
    float max = energyAtEachIteration.maxCoeff();
    cv::Mat energyPlot = cv::Mat::zeros( 500, len*XSPACING + 40, CV_8UC3 ) + cv::Scalar(255,255,255);

    // draw axes
    cv::line( energyPlot, cv::Point(20,0), cv::Point(20,energyPlot.rows), cv::Scalar(0,0,0) ); //y-axis
    cv::line( energyPlot, cv::Point(0,energyPlot.rows-20), cv::Point(XSPACING*len+40, energyPlot.rows-20), cv::Scalar(0,0,0) ); //x-axis

    // mark axes
    for( int p=0 ; p<len ; p+=10 ) { // plot's x-axis
        char str[40];
        sprintf( str, "%d", p );
        cv::circle( energyPlot, cv::Point(XSPACING*p+20, energyPlot.rows-20),2, cv::Scalar(0,0,255), -1 );
        cv::putText( energyPlot, str, cv::Point(XSPACING*p+20, energyPlot.rows-10), cv::FONT_HERSHEY_COMPLEX_SMALL, .5, cv::Scalar(0,0,0));
    }
    for( float p=0.0 ; p<100000.0 ; p+=5000.0 ) { //plot y-axis labels
        char str[40];
        sprintf( str, "%2.0f00", (int) p/1000.0 );
        cv::circle( energyPlot, cv::Point(20,energyPlot.rows - 20 - p/200.0),2, cv::Scalar(0,0,255), -1);
        cv::putText( energyPlot, str, cv::Point(3, energyPlot.rows - 25 - p/200.0), cv::FONT_HERSHEY_COMPLEX_SMALL, .5, cv::Scalar(0,0,0) );
    }


    // plot energy bars
    for( int i=0 ; i<energyAtEachIteration.rows() ; i++ )
    {
        float energy = energyAtEachIteration(i);
        cv::line( energyPlot, cv::Point(XSPACING*i+20, energyPlot.rows-20), cv::Point(XSPACING*i+20, energyPlot.rows - 20 - energy/200.0), cv::Scalar(255,0,0));
    }

    if( bestEnergyIndex >= 0 ) {
    float energy = energyAtEachIteration(bestEnergyIndex);
    cv::line( energyPlot, cv::Point(XSPACING*bestEnergyIndex+20, energyPlot.rows-20), cv::Point(XSPACING*bestEnergyIndex+20, energyPlot.rows - 20 - energy/200.0), cv::Scalar(0,255,0));
    }



    cv::imshow( "energy progress", energyPlot );
}


void SolveDVO::visualizeReprojectedDepth(Eigen::MatrixXf &eim, Eigen::MatrixXf &reprojDepth)
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


    FColorMap fcm(64);

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

            xim.at<cv::Vec3b>(i,j) = fcm.at(colorIndx); //colors[colorIndx];
        }
    }

    cv::imshow( "reprojected depth heatmaps", xim );



}


/// @brief Computes the distance transform of the reference frame (all levels) and stores in std::vector
void SolveDVO::computeDistTransfrmOfRef()
{
    assert( isRefFrameAvailable && "Reference frames need to be available in order to compute dist transform on them" );

    ref_distance_transform.clear();
    ref_edge_map.clear();
    ref_DT_gradientX.clear();
    ref_DT_gradientY.clear();


    isRefDistTransfrmAvailable = false;
    //for each level
    for( int lvl=0 ; lvl<im_r.size() ; lvl++ )
    {
        Eigen::MatrixXf ref_t = im_r[lvl];
        ///// DISTANCE TRANSFORM of ref_t
        //ROS_INFO( "distance transform (level=%d)", lvl );
        cv::Mat refCvMat, refEdge, refDistTransCvMat;
        cv::eigen2cv( ref_t, refCvMat );
        // Sobel
//        cv::Sobel( refCvMat, refEdge, CV_8U, 1, 1 );
//        refEdge = 255 - refEdge;
//        cv::threshold( refEdge, refEdge, 250, 255, cv::THRESH_BINARY );

        // Canny
        refCvMat.convertTo(refCvMat, CV_8U);
        cv::Canny( refCvMat, refEdge, 150, 100, 3, true );
        refEdge = 255 - refEdge;

//        cv::distanceTransform( refEdge, refDistTransCvMat, CV_DIST_L2, 5 );
        cv::distanceTransform( refEdge, refDistTransCvMat, CV_DIST_L2, CV_DIST_MASK_PRECISE );

#ifdef __SCALE_NORMALIZE_DISTANCE_TRANFROM
        cv::normalize(refDistTransCvMat, refDistTransCvMat, 0.0, 255.0, cv::NORM_MINMAX);
#endif

//        double min, max;
//        cv::minMaxLoc(refDistTransCvMat, &min, &max);
        //ROS_INFO_STREAM( "min : "<< min << " max : "<< max << " dataTYpe : "<< cvMatType2str(refDistTransCvMat.type()) );
        ///// END DISTANCE TRANSFORM

        Eigen::MatrixXf refDistTrans;
        Eigen::MatrixXi refEdgeMap;
        cv::cv2eigen(refDistTransCvMat, refDistTrans);
        refEdge = 255 - refEdge; //done for visualization
        cv::cv2eigen(refEdge, refEdgeMap);

        // grad of dist transform
        Eigen::MatrixXf dGx, dGy; //image gradient
        imageGradient(refDistTrans, dGx, dGy);
        ref_DT_gradientX.push_back(dGx);
        ref_DT_gradientY.push_back(dGy);


        ref_distance_transform.push_back(refDistTrans);
        ref_edge_map.push_back(refEdgeMap);
    }

    isRefDistTransfrmAvailable = true;
}

void SolveDVO::computeDistTransfrmOfNow()
{
    assert( isNowFrameAvailable && "Now frames need to be available in order to compute dist transform on them" );

    now_distance_transform.clear();
    now_edge_map.clear();
    now_DT_gradientX.clear();
    now_DT_gradientY.clear();


    isNowDistTransfrmAvailable = false;
    //for each level
    for( int lvl=0 ; lvl<im_n.size() ; lvl++ )
    {
        Eigen::MatrixXf now_t = im_n[lvl];
        ///// DISTANCE TRANSFORM of ref_t
        //ROS_INFO( "distance transform (level=%d)", lvl );
        cv::Mat nowCvMat, nowEdge, nowDistTransCvMat;
        cv::eigen2cv( now_t, nowCvMat );

        // Sobel
//        cv::Sobel( nowCvMat, nowEdge, CV_8U, 1, 1 );
//        nowEdge = 255 - nowEdge;
//        cv::threshold( nowEdge, nowEdge, 250, 255, cv::THRESH_BINARY );

        // Canny
        nowCvMat.convertTo(nowCvMat, CV_8U);
        cv::Canny( nowCvMat, nowEdge, 150, 100, 3, true );
        nowEdge = 255 - nowEdge;

//        cv::distanceTransform( nowEdge, nowDistTransCvMat, CV_DIST_L2, 5 );
        cv::distanceTransform( nowEdge, nowDistTransCvMat, CV_DIST_L2, CV_DIST_MASK_PRECISE );

#ifdef __SCALE_NORMALIZE_DISTANCE_TRANFROM
        cv::normalize(nowDistTransCvMat, nowDistTransCvMat, 0.0, 255.0, cv::NORM_MINMAX);
#endif

//        double min, max;
//        cv::minMaxLoc(nowDistTransCvMat, &min, &max);
        //ROS_INFO_STREAM( "(Now)min : "<< min << " max : "<< max << " dataTYpe : "<< cvMatType2str(nowDistTransCvMat.type()) );
        ///// END DISTANCE TRANSFORM

        Eigen::MatrixXf nowDistTrans;
        Eigen::MatrixXi nowEdgeMap;
        cv::cv2eigen(nowDistTransCvMat, nowDistTrans);
        nowEdge = 255 - nowEdge; //done for visualization
        cv::cv2eigen(nowEdge, nowEdgeMap);

        // grad of dist transform
        Eigen::MatrixXf dGx, dGy; //image gradient
        imageGradient(nowDistTrans, dGx, dGy);
        now_DT_gradientX.push_back(dGx);
        now_DT_gradientY.push_back(dGy);

        now_distance_transform.push_back(nowDistTrans);
        now_edge_map.push_back(nowEdgeMap);
    }

    isNowDistTransfrmAvailable = true;
}


/// @brief Only loop through the images received (now frame) and optionally GT. No DVO estimation.
void SolveDVO::loopDry()
{
#ifdef __TF_GT__
    ROS_INFO( "Will Also publish GT pose (wrt 1st frame) from tf-data");
    tf::TransformListener tflis;
    Eigen::Matrix3f _tf_Rf, _tf_Rc, _tf_Rc_f;
    Eigen::Vector3f _tf_Tf, _tf_Tc, _tf_Tc_f;
    bool _tf_1st_frame = false;
#endif //__TF_GT__

#ifdef __DATA_FROM_XML_FILES__
    char frameFileName[1000];
#endif //__DATA_FROM_XML_FILES__


    // Dry Loop
    long nFrame = 0;
    ros::Rate rate(30);
    while( ros::ok() )
    {
        ros::spinOnce();

#ifdef __DATA_FROM_XML_FILES__
        int iDataFrameNum = __DATA_FROM_XML_FILES__START + nFrame;
        if( iDataFrameNum > __DATA_FROM_XML_FILES__END )
        {
            ROS_ERROR( "Done with all files...Quitting..." );
            break;
        }
        sprintf( frameFileName, "%s/framemono_%04d.xml", __DATA_FROM_XML_FILES__, iDataFrameNum );
        bool flag = loadFromFile(frameFileName );
        if( flag == false ) {
            ROS_ERROR( "No More files, Quitting..");
            break;
        }
        else
            ROS_INFO( "=== Loaded file ``%s'' ===", frameFileName );

#endif //__DATA_FROM_XML_FILES__

        if( !(this->isFrameAvailable) )
            continue;



        setRcvdFrameAsNowFrame();


#ifdef __TF_GT__
        tf::StampedTransform transform;
        try{
            tflis.lookupTransform("/world", "/openni_rgb_optical_frame", ros::Time(0), transform );

            if( _tf_1st_frame == false )
            {
                tfTransform2EigenMat(transform, _tf_Rf, _tf_Tf);
                _tf_1st_frame = true;
            }

            if( _tf_1st_frame == true )
            {
                tfTransform2EigenMat(transform, _tf_Rc, _tf_Tc);
                _tf_Tc_f = _tf_Rf.transpose() * ( _tf_Tc - _tf_Tf );
                _tf_Rc_f = _tf_Rf.transpose() * _tf_Rc;

                geometry_msgs::Pose __currentGTPose;
                mviz.publishFromTF(_tf_Rc_f, _tf_Tc_f, __currentGTPose);
            }
        }
        catch(tf::TransformException &ex) {
            ROS_ERROR("NO TF Message");
        }

#endif


        imshowEigenImage( "current Frame", im_n[0] );
        char ch = cv::waitKey(1);
        if( ch == 27 ){ // ESC
            ROS_ERROR( "ESC pressed quitting...");
            exit(1);
        }
        isFrameAvailable = false;
        rate.sleep();
        nFrame++;

    }

}


/// @brief The event loop. Basically an ever running while with ros::spinOnce()
/// This is a re-implementation taking into care the memory scopes and also processing only points with higher image gradients
void SolveDVO::loop()
{



#ifdef __TF_GT__
    ROS_INFO( "Will also publish GT pose (wrt 1st frame) from tf-data");
    // "*c" --> absolute pose of current frame (wrt to MOCAP)
    // "*f" --> pose of 1st frame (wrt to MOCAP)
    // "*c_f" --> pose of current frame wrt to 1st frame
    tf::TransformListener tflis;
    Eigen::Matrix3f _tf_Rf = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f _tf_Rc = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f _tf_Rc_f = Eigen::Matrix3f::Identity();
    Eigen::Vector3f _tf_Tf = Eigen::Vector3f::Zero();
    Eigen::Vector3f _tf_Tc = Eigen::Vector3f::Zero();
    Eigen::Vector3f _tf_Tc_f = Eigen::Vector3f::Zero();
    bool _tf_GT_1_frame_available = false;

    std::vector<float>drifts;
    drifts.reserve(10000);
#endif //__TF_GT__


#ifdef __DATA_FROM_XML_FILES__
    char frameFileName[1000];
#endif //__DATA_FROM_XML_FILES__

    double iterationsComputeTime=0.0;
    double avgIterationsTime=0.0;


    // Pose of now frame wrt currently set reference frame
    Eigen::Matrix3f cR = Eigen::Matrix3f::Identity();
    Eigen::Vector3f cT = Eigen::Vector3f::Zero();
    Eigen::Matrix3d cR_64 = Eigen::Matrix3d::Identity();
    Eigen::Vector3d cT_64 = Eigen::Vector3d::Zero();


    // Additional data returned by Gauss-Newton iterations
    Eigen::VectorXf epsilonVec;
    Eigen::MatrixXf reprojections;
    Eigen::VectorXf energyAtEachIteration;
    int bestEnergyIndex = -1;
    float visibleRatio = 0.0;


    long lastRefFrame=0;

    ros::Rate rate(35);
    long nFrame=0;

    //
    // =================================  Capture the 1st frame (as ref-frame) unconditionally ==============================
    while( ros::ok() )
    {
        ros::spinOnce();
#ifdef __DATA_FROM_XML_FILES__
        int iDataFrameNum = __DATA_FROM_XML_FILES__START + nFrame;
        if( iDataFrameNum > __DATA_FROM_XML_FILES__END )
        {
            ROS_ERROR( "Done with all files...Quitting..." );
            break;
        }
        sprintf( frameFileName, "%s/framemono_%04d.xml", __DATA_FROM_XML_FILES__, iDataFrameNum );
        bool flag = loadFromFile(frameFileName );
        if( flag == false ) {
            ROS_ERROR( "No More files, Quitting..");
            break;
        }
        else
            ROS_INFO( "=== Loaded file ``%s'' ===", frameFileName );

#endif //__DATA_FROM_XML_FILES__

        ROS_INFO_ONCE( "Waiting for RGBDFramePyD messages..." );
        if( (this->isFrameAvailable) )
        {
            ROS_INFO( "Setting 1st Frame as reference frame" );
            setRcvdFrameAsRefFrame();
            preProcessRefFrame();
            lastRefFrame = 0;





#ifdef __TF_GT__
            //note down the pose of 1st frame of GT
        tf::StampedTransform transform;
        try{
             //block until it is available. maximum wait for 1 sec for tf to be active
            tflis.waitForTransform("/world", "/openni_rgb_optical_frame", ros::Time(0), ros::Duration(1.0) );

            tflis.lookupTransform("/world", "/openni_rgb_optical_frame", ros::Time(0), transform );

            tfTransform2EigenMat(transform, _tf_Rf, _tf_Tf);

            tfTransform2EigenMat(transform, _tf_Rc, _tf_Tc);
            _tf_Tc_f = _tf_Rf.transpose() * ( _tf_Tc - _tf_Tf );
            _tf_Rc_f = _tf_Rf.transpose() * _tf_Rc;

            ROS_INFO( "Registered 1st GT frame transform" );
            _tf_GT_1_frame_available = true;


            geometry_msgs::Pose __currentGTPose;
            mviz.publishFromTF(_tf_Rc_f, _tf_Tc_f, __currentGTPose);
        }
        catch(tf::TransformException &ex) {
            ROS_ERROR("NO TF Message. Will set one of the subsequent frame as the 1st frame. Everything might be slightly off");
            ROS_ERROR("%s",ex.what());
        }


        if( _tf_GT_1_frame_available==false ) //make a blue sphere in case of slight delay in tf
            gop.pushAsKeyFrame(nFrame, 10, cR_64, cT_64 );
        else
#endif //__TF_GT__

        // setting 1st frame as key-frame with itself as origin.
        gop.pushAsKeyFrame(nFrame, 1, cR_64, cT_64 );

            this->isFrameAvailable = false;
            nFrame++;
            ROS_INFO( "Done setting 1st received frame as reference..now will continue processing frames" );
            signalGetNewRefImage = false;
            rate.sleep();
            break;
        }
        rate.sleep();
    }


    //
    // ========================= Process ALL OTHER Frames ===============================
    while( ros::ok() )
    {
        ros::spinOnce();
#ifdef __DATA_FROM_XML_FILES__
        int iDataFrameNum = __DATA_FROM_XML_FILES__START + __DATA_SKIP_FACTOR*nFrame;
        if( iDataFrameNum > __DATA_FROM_XML_FILES__END )
        {
            ROS_ERROR( "Done with all files...Quitting..." );
            break;
        }
        sprintf( frameFileName, "%s/framemono_%04d.xml", __DATA_FROM_XML_FILES__, iDataFrameNum );
        bool flag = loadFromFile(frameFileName );
        if( flag == false ) {
            ROS_ERROR( "No More files, Quitting..");
            break;
        }
        else
            ROS_INFO( "=== Loaded file ``%s'' ===", frameFileName );

#endif //__DATA_FROM_XML_FILES__

        if( !(this->isFrameAvailable) )
            continue;

        ROS_INFO( "=-=-=-=-=-=-== Frame # %ld ==-=-=-=-=-=-=", nFrame );


        // =========== Set Now Frame ==============

        setRcvdFrameAsNowFrame();


#ifdef __TF_GT__
        // read the pose from TF
        tf::StampedTransform transform;
        try{
            tflis.lookupTransform("/world", "/openni_rgb_optical_frame", ros::Time(0), transform );

            if( _tf_GT_1_frame_available == false )
            {
                tfTransform2EigenMat(transform, _tf_Rf, _tf_Tf);
                _tf_GT_1_frame_available = true;
            }


            if( _tf_GT_1_frame_available ) {
            tfTransform2EigenMat(transform, _tf_Rc, _tf_Tc);
            _tf_Tc_f = _tf_Rf.transpose() * ( _tf_Tc - _tf_Tf );
            _tf_Rc_f = _tf_Rf.transpose() * _tf_Rc;
            }
        }
        catch(tf::TransformException &ex) {
            ROS_ERROR("NO TF Message");
            ROS_ERROR("%s",ex.what());

        }
#endif //__TF_GT__


        // ================== Run Iterations ==================
        ros::Time jstart = ros::Time::now();
//        runIterations(2, 50, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio  );
//        runIterations(1, 100, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio );
//        runIterations(0, 300, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio );

        for( int f=(iterationsConfig.size()-1) ; f>=0 ; f-- )
        {
            if( iterationsConfig[f] > 0 )
            {
                ROS_INFO( "Run %d iterations @ level %d", iterationsConfig[f], f );
                runIterations(f, iterationsConfig[f], cR_64, cT_64, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio );
            }
        }

        ros::Duration jdur = ros::Time::now() - jstart;
        iterationsComputeTime = jdur.toSec();
        avgIterationsTime = avgIterationsTime*(nFrame-1)/nFrame + iterationsComputeTime/nFrame;
        ROS_INFO( "Iterations done in %lf ms", iterationsComputeTime*1000 );

        //mviz.publishFullPointCloud();

        // This was a critical-patch. Fixed a critical bug. The exit conditions need to be checked on the f_best and not all the epsilons.
        // Before this was done in gaussNewtonIteration which was obiviously not correct as it checkd the threshold for the last iteration
        // and not the best iteration. Similar fix need to be done with %-visibility condition.
#ifdef __ENABLE_DISPLAY__
        float b_cap = processResidueHistogram( epsilonVec, false );
#else
        float b_cap = processResidueHistogram( epsilonVec, true );
#endif //__ENABLE_DISPLAY__

        int reasonForChange=0;


        // ================== Change KeyFrame ? ====================
        //   Check if the estimation was bad and if there is a need to change key-frame


        /*
        // These 3 following IFs will check if it is time to change the reference frame
        if( b_cap > laplacianThreshExitCond ) {
            ROS_ERROR( "Fitted laplacian b: %.2f. Laplacian b_thresh : %.2f. Signal change of reference frame", b_cap, laplacianThreshExitCond );
            snprintf( signalGetNewRefImageMsg, 450, "Fitted laplacian b: %.2f. Laplacian b_thresh : %.2f. Signal change of reference frame", b_cap, laplacianThreshExitCond );
            signalGetNewRefImage = true;
            reasonForChange=2;
        }


        if( visibleRatio < ratio_of_visible_pts_thresh ) {
            ROS_ERROR( "Only %.2f of tracked points visible. Required %.2f. Signal change of reference frame", visibleRatio, ratio_of_visible_pts_thresh);
            snprintf(signalGetNewRefImageMsg, 450, "Only %.2f of tracked points visible. Required %.2f. Signal change of reference frame", visibleRatio, ratio_of_visible_pts_thresh );
            signalGetNewRefImage = true;
            reasonForChange=3;
        }

        if( reprojections.cols() < 50 ) //this was put in here after experience from TUM-RGBD/kidnap sequence
        {
            ROS_ERROR( "Too few reprojected points, change reference frame" );
            signalGetNewRefImage = true;
            reasonForChange=4;
        }
*/


        signalGetNewRefImage = false;
        if( (nFrame - lastRefFrame) == 5 )
        {
            signalGetNewRefImage = true;
            reasonForChange=5;
        }





#ifdef __OLD__REF_UPDATE
        if( signalGetNewRefImage == true )
        //if( (nFrame % 5) == 0 )
        {
            ROS_INFO( "!! NEW REFERENCE FRAME !!");
            ROS_INFO( "Reason : %s", signalGetNewRefImageMsg );


            lastRefFrame = nFrame;
            setRcvdFrameAsRefFrame();
            preProcessRefFrame();

            gop.pushAsKeyFrame(nFrame, reasonForChange, cR, cT );

            // reset pose-guess with respect to newly changed reference frame
            cR = Eigen::Matrix3f::Identity();
            cT = Eigen::Vector3f::Zero();
            signalGetNewRefImage = false;
            sprintf( signalGetNewRefImageMsg, "" );
        }
        else
        {
            //Currently estimated pose was not of a keyframe
            gop.pushAsOrdinaryFrame(nFrame, cR, cT);
        }
#endif


#ifdef __NEW__REF_UPDATE
        //currently estimate cR and cT are not as good.
        // the 2nd condition here says that if 'n-1'th frame was already a reference frame than turn a blind eye to the warning on
        // cR, cT being bad
        if( (signalGetNewRefImage == true) && (lastRefFrame!=(nFrame-1)) )
        {
            // Set "n-1" as ref frame
            lastRefFrame = nFrame-1;
            setPrevFrameAsRefFrame();
            preProcessRefFrame();


            // Update "most recently" pushed GOP-frame as a keyframe
            gop.updateMostRecentToKeyFrame(reasonForChange);

            // reset cR, cT
            cR_64 = Eigen::Matrix3d::Identity();
            cT_64 = Eigen::Vector3d::Zero();
            signalGetNewRefImage = false;
            sprintf( signalGetNewRefImageMsg, "" );

            //rerun iterations
//            runIterations(2, 50, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio  );
//            runIterations(1, 100, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio );
//            runIterations(0, 300, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio );

            for( int f=(iterationsConfig.size()-1) ; f>=0 ; f-- )
            {
                if( iterationsConfig[f] > 0 )
                {
                    ROS_INFO( "re-Run %d iterations @ level %d", iterationsConfig[f], f );
                    runIterations(f, iterationsConfig[f], cR_64, cT_64, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio );
                }
            }

            // push GOP for now frame as ordinary frame
//            cR_64 = cR.cast<double>();
//            cT_64 = cT.cast<double>();
            gop.pushAsOrdinaryFrame(nFrame, cR_64, cT_64);
        }
        else
        {
            //Currently estimated pose was acceptable wrt to lastRefFrame so push as ordinary
//            cR_64 = cR.cast<double>();
//            cT_64 = cT.cast<double>();
            gop.pushAsOrdinaryFrame(nFrame, cR_64, cT_64);
        }
#endif






        //
        // ============== PUBLISHING ==============
        //

        ros::Time pubstart = ros::Time::now();
        geometry_msgs::Pose __currentEstimatedPose;
        mviz.publishGOP(__currentEstimatedPose);
#ifdef __WRITE_EST_POSE_TO_FILE
        printPose(__currentEstimatedPose, "main/__currentEstPose ", estPoseFile );
#else
        printPose(__currentEstimatedPose, "main/__currentEstPose ", std::cout );
#endif

        //mviz.publishFullPointCloud();
        ros::Duration pubdur = ros::Time::now() - pubstart;
        double displayTime = pubdur.toSec();
        //ROS_INFO( "Display done in %lf ms", displayTime*1000 );




#ifdef __TF_GT__
        geometry_msgs::Pose __currentGTPose;
        mviz.publishFromTF(_tf_Rc_f, _tf_Tc_f, __currentGTPose );
#ifdef __WRITE_GT__POSE_TO_FILE
        printPose(__currentGTPose, "main/__currentGTPose  ", gtPoseFile );
#else
        printPose(__currentGTPose, "main/__currentGTPose  ", std::cout );
#endif

        float drift = getDriftFromPose( __currentGTPose, __currentEstimatedPose );
        drifts.push_back(drift);
        analyzeDriftVector( drifts );
#endif //__TF_GT__

        // ================ ALL DISPLAY ==============
        {
#ifdef __ENABLE_DISPLAY__
        //
        //Debugging display of re-projected points


            int xlevel = 0;
            Eigen::MatrixXi reprojectedMask = Eigen::MatrixXi::Zero(im_n[xlevel].rows(), im_n[xlevel].cols());
            cordList_2_mask(reprojections, reprojectedMask);
            cv::Mat outIm,outIm2, outImGray, outRef;
            sOverlay(now_distance_transform[xlevel], reprojectedMask, outIm, cv::Vec3b(0,255,0));

//            if( isPrevFrameAvailable ) {
//                imshowEigenImage("n-1", p_now_framemono[xlevel]);
//                imshowEigenImage("n", im_n[xlevel]);
//            }

            sOverlay(im_n[xlevel], reprojectedMask, outImGray, cv::Vec3b(0,255,0));

            sOverlay(im_r[xlevel], ref_edge_map[xlevel], outRef, cv::Vec3b(0,0,255));

            cv::imshow( "reprojection with cR,cT (on now dist-tran", outIm);
            //cv::imshow( "reprojection with cR,cT (on now gray", outImGray);
            cv::imshow( "selected edges on ref", outRef);
            visualizeDistanceResidueHeatMap(im_n[xlevel], reprojectedMask, now_distance_transform[xlevel] );


            //printFrameIndex2Scratch(debugScratchBoard, nFrame, lastRefFrame, lastRefFrameComputeTime, gaussNewtonIterationsComputeTime, true );
            //cv::imshow( "scratBoard", debugScratchBoard );

            visualizeEnergyProgress( energyAtEachIteration, bestEnergyIndex, (energyAtEachIteration.rows() < 300)?4:1 );



            char ch = cv::waitKey(__ENABLE_DISPLAY__);


            if( ch == 27 ){ // ESC
                ROS_ERROR( "ESC pressed quitting...");
                exit(1);
            }


#endif
        }


        // MINIMAL DISPLAY
        {
#ifdef __MINIMAL_DISPLAY
        int xlevel = 0;
        Eigen::MatrixXi reprojectedMask = Eigen::MatrixXi::Zero(im_n[xlevel].rows(), im_n[xlevel].cols());
        cordList_2_mask(reprojections, reprojectedMask);

        cv::Mat outRef;
        sOverlay(im_r[xlevel], ref_edge_map[xlevel], outRef, cv::Vec3b(0,0,255));
        cv::imshow( "selected edges on ref", outRef);


        visualizeDistanceResidueHeatMap(im_n[xlevel], reprojectedMask, now_distance_transform[xlevel] );

        visualizeEnergyProgress( energyAtEachIteration, bestEnergyIndex, (energyAtEachIteration.rows() < 300)?2:1 );


        char ch = cv::waitKey(__MINIMAL_DISPLAY);
        if( ch == 27 ){ // ESC
            ROS_ERROR( "ESC pressed quitting...");
            ROS_ERROR( "Average iterations time per frame : %lf ms", avgIterationsTime*1000.0 );
            exit(1);
        }

#endif //__MINIMAL_DISPLAY
    }

//        ros::Time debug_start = ros::Time::now();
//        mviz.debug(cR, cT);
//        ros::Duration debug_end = ros::Time::now() - debug_start;
//        double debugTime = debug_end.toSec();
//        ROS_INFO( "Debug function done in %lf ms", debugTime*1000 );


        isFrameAvailable = false; //this is put in place to process at-max the arrival rate.

        rate.sleep();
        nFrame++;
        ros::spinOnce();

    }

}


/// Tmperory function to study effect of heavy ball
void SolveDVO::casualTestFunction()
{
    char frameFileName[500];
    bool flag;


    Eigen::Matrix3d cR_64 = Eigen::Matrix3d::Identity();
    Eigen::Vector3d cT_64 = Eigen::Vector3d::Zero();


    // Additional data returned by Gauss-Newton iterations
    Eigen::VectorXf epsilonVec;
    Eigen::MatrixXf reprojections;
    Eigen::VectorXf energyAtEachIteration;
    int bestEnergyIndex = -1;
    float visibleRatio = 0.0;


    // Set a ref frame
    sprintf( frameFileName, "%s/framemono_%04d.xml", "TUM_RGBD/fr1_rpy", 80 );
    flag = loadFromFile(frameFileName );
    if( flag == false ) {
        ROS_ERROR( "Cannot open file1");
    }
    else
        ROS_INFO( "=== Loaded file ``%s'' ===", frameFileName );

    setRcvdFrameAsRefFrame();
    preProcessRefFrame();





    // Set a now frame
    sprintf( frameFileName, "%s/framemono_%04d.xml", "TUM_RGBD/fr1_rpy", 85 );
    flag = loadFromFile(frameFileName );
    if( flag == false ) {
        ROS_ERROR( "Cannot open file1");
    }
    else
        ROS_INFO( "=== Loaded file ``%s'' ===", frameFileName );

    setRcvdFrameAsNowFrame();




    // Run iterations
    runIterations(0, 100, cR_64, cT_64, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio );




    imshowEigenImage( "current Frame", im_n[0] );
    char ch = cv::waitKey(0);
    if( ch == 27 ){ // ESC
        ROS_ERROR( "ESC pressed quitting...");
        exit(1);
    }

    for( int i=0 ; i<energyAtEachIteration.rows() ; i++ )
    {
        std::cout<< energyAtEachIteration[i] << std::endl;
    }
}

/*

/// @brief The event loop. Basically an ever running while with ros::spinOnce()
/// This is a re-implementation taking into care the memory scopes and also processing only points with higher image gradients
void SolveDVO::loopFromFile()
{


//    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
//        ros::console::notifyLoggerLevelsChanged();

    char frameFileName[50];
    //const char * folder = "xdump_right2left"; //hard dataset
    //const char * folder = "xdump_left2right";
    //const char * folder = "TUM_RGBD/fr1_xyz";
    const char * folder = "TUM_RGBD/fr2_desk";

    const int START = 300;
    const int END = 3000;


    long lastRefFrame=0;
    double lastRefFrameComputeTime;
    double gaussNewtonIterationsComputeTime;



    Eigen::Matrix3f cR = Eigen::Matrix3f::Identity();
    Eigen::Vector3f cT = Eigen::Vector3f::Zero();
    Eigen::VectorXf epsilonVec;
    Eigen::MatrixXf reprojections;
    Eigen::VectorXf energyAtEachIteration;
    int bestEnergyIndex = -1;
    float visibleRatio = 0.0;


    // Pose of currently set Reference-frame (key frame)
    Eigen::Matrix3f keyR = Eigen::Matrix3f::Identity();
    Eigen::Vector3f keyT = Eigen::Vector3f::Zero();


    // Pose of now frame in global frame of reference
    Eigen::Matrix3f nR = Eigen::Matrix3f::Identity();
    Eigen::Vector3f nT = Eigen::Vector3f::Zero();





    ros::Rate rate(30);
    for( int iFrameNum = START; iFrameNum < END ; iFrameNum++ )
    {

        sprintf( frameFileName, "%s/framemono_%04d.xml", folder, iFrameNum );
        bool flag = loadFromFile(frameFileName );
        if( flag == false ) {
            ROS_ERROR( "No More files, Quitting..");
            break;
        }


        //if( signalGetNewRefImage == true )
        if( iFrameNum%5 == 0 )
        {
            ROS_INFO( "!! NEW REFERENCE FRAME !!");
            ROS_INFO( "Reason : %s", signalGetNewRefImageMsg );

            if( iFrameNum > START ) {
                setRcvdFrameAsNowFrame();
            // before changing the reference-frame estimate itz pose
//            gaussNewtonIterations(2, 20, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio  );
//            gaussNewtonIterations(1, 20, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio );
            runIterations(0, 500, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio );
            }

            keyR = nR;
            keyT = nT;

            lastRefFrame = iFrameNum;
            setRcvdFrameAsRefFrame();

            preProcessRefFrame();

            cR = Eigen::Matrix3f::Identity();
            cT = Eigen::Vector3f::Zero();


            signalGetNewRefImage = false;

        }
        //else
        //{
            setRcvdFrameAsNowFrame();

            ros::Time jstart = ros::Time::now();
            runIterations(0, 500, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio );
            ros::Duration jdur = ros::Time::now() - jstart;
            gaussNewtonIterationsComputeTime = jdur.toSec();
            ROS_INFO( "Iterations done in %lf ms", gaussNewtonIterationsComputeTime*1000 );

        //}


            nT = keyT + keyR*cT;
            nR = keyR*cR;



            // This was a critical-patch. Fixed a critical bug. The exit conditions need to be checked on the f_best and not all the epsilons.
            // Before this was done in gaussNewtonIteration which was obiviously not correct as it checkd the threshold for the last iteration
            // and not the best iteration. Similar fix need to be done with %-visibility condition.
#ifdef __ENABLE_DISPLAY__
            float b_cap = processResidueHistogram( epsilonVec, false );
#else
            float b_cap = processResidueHistogram( epsilonVec, true );
#endif //__ENABLE_DISPLAY__



            if( b_cap > laplacianThreshExitCond ) {
                ROS_ERROR( "Fitted laplacian b: %.2f. Laplacian b_thresh : %.2f. Signal change of reference frame", b_cap, laplacianThreshExitCond );
                snprintf( signalGetNewRefImageMsg, 450, "Fitted laplacian b: %.2f. Laplacian b_thresh : %.2f. Signal change of reference frame", b_cap, laplacianThreshExitCond );
                signalGetNewRefImage = true;
            }


            if( visibleRatio < ratio_of_visible_pts_thresh ) {
                ROS_ERROR( "Only %.2f of tracked points visible. Required %.2f. Signal change of reference frame", visibleRatio, ratio_of_visible_pts_thresh);
                snprintf(signalGetNewRefImageMsg, 450, "Only %.2f of tracked points visible. Required %.2f. Signal change of reference frame", visibleRatio, ratio_of_visible_pts_thresh );
                signalGetNewRefImage = true;
            }





#ifdef __ENABLE_DISPLAY__
            //
            // DISPLAY
            //printRT( cR, cT, "Updated cR,cT");
            {
                int xlevel = 0;
            Eigen::MatrixXi reprojectedMask = Eigen::MatrixXi::Zero(im_n[xlevel].rows(), im_n[xlevel].cols());
            cordList_2_mask(reprojections, reprojectedMask);
            cv::Mat outIm, outImGray, outRef;
            sOverlay(now_distance_transform[xlevel], reprojectedMask, outIm, cv::Vec3b(0,255,0));
            sOverlay(im_n[xlevel], reprojectedMask, outImGray, cv::Vec3b(0,255,0));

            sOverlay(im_r[xlevel], ref_edge_map[xlevel], outRef, cv::Vec3b(0,0,255));

            cv::imshow( "reprojection with cR,cT (on now dist-tran", outIm);
            cv::imshow( "reprojection with cR,cT (on now gray", outImGray);
            cv::imshow( "selected edges on ref", outRef);
            visualizeDistanceResidueHeatMap(im_n[xlevel], reprojectedMask, now_distance_transform[xlevel] );

            visualizeEnergyProgress( energyAtEachIteration, bestEnergyIndex, (energyAtEachIteration.rows() < 300)?4:1 );


            processResidueHistogram( epsilonVec, false );
            if( iFrameNum == START ) { //only attempt to move windows in 1st iteration
            cv::moveWindow("reprojection with cR,cT (on now dist-tran", 800, 600 );
            cv::moveWindow("reprojection with cR,cT (on now gray"     , 1150, 600 );
            cv::moveWindow("selected edges on ref", 1500, 600 );
            }
            char ch = cv::waitKey(__ENABLE_DISPLAY__);


            ROS_ERROR( "End of Iterations Display.....!");
            }

            //
            // END DISPLAY

#endif


#ifdef __MINIMAL_DISPLAY
            {
                int xlevel = 0;
                Eigen::MatrixXi reprojectedMask = Eigen::MatrixXi::Zero(im_n[xlevel].rows(), im_n[xlevel].cols());
                cordList_2_mask(reprojections, reprojectedMask);

                cv::Mat outRef;
                sOverlay(im_r[xlevel], ref_edge_map[xlevel], outRef, cv::Vec3b(0,0,255));
                cv::imshow( "selected edges on ref", outRef);


                visualizeDistanceResidueHeatMap(im_n[xlevel], reprojectedMask, now_distance_transform[xlevel] );


                visualizeEnergyProgress( energyAtEachIteration, bestEnergyIndex, (energyAtEachIteration.rows() < 300)?4:2 );

                char ch = cv::waitKey(__MINIMAL_DISPLAY);
                if( ch == 27 ){ // ESC
                    ROS_ERROR( "ESC pressed quitting...");
                    break;
                }
            }

#endif //__MINIMAL_DISPLAY


            //
            // Publishing to RVIZ
            //publishPoseFinal(nR, nT);
            //publishReferencePointCloud(1);
            //mviz.incrementalSphere();
            //mviz.publishCurrentPointCloud(1);
            mviz.publishPoseFinal(nR, nT);
            mviz.publishPath();




            ros::spinOnce();
            rate.sleep();



            //
            // OpenCV WaitKey()
#ifdef __ENABLE_DISPLAY__
            char ch = cv::waitKey(__ENABLE_DISPLAY__);
            if( ch == 27 ){ // ESC
                ROS_ERROR( "ESC pressed quitting...");
                break;
            }

#endif //__ENABLE_DISPLAY__

    }

    ROS_INFO( "Normal Exit...:)");
}
*/



/// @brief Given the input transform in tf format, returns the Eigen R,T matrices
void SolveDVO::tfTransform2EigenMat( tf::StampedTransform& tr, Eigen::Matrix3f& R, Eigen::Vector3f& T )
{
    Eigen::Quaternionf Qf(tr.getRotation().getW(), tr.getRotation().getX(), tr.getRotation().getY(), tr.getRotation().getZ() );
    R = Qf.toRotationMatrix();
    T(0) = tr.getOrigin().getX();
    T(1) = tr.getOrigin().getY();
    T(2) = tr.getOrigin().getZ();
}



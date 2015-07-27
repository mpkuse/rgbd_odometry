#include <SolveDVO.h>


/// default constructor. Init the subscribers
SolveDVO::SolveDVO()
{
    isCameraIntrinsicsAvailable = false;
    isFrameAvailable = false;
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
    laplacianThreshExitCond = 10.0f;
    psiNormTerminationThreshold = 1.0E-7;



    //
    // Setting up Publishers & Subscribers

    //sub = nh.subscribe( "odometry/rgbd", 2, &SolveDVO::imageArrivedCallBack, this );
    sub = nh.subscribe( "Xtion/rgbdPyramid", 10, &SolveDVO::imageArrivedCallBack, this );

  

    //
    // Visualization Class
    mviz.setNodeHandle(this); //this call also setups the publisher
    mviz.setRVizFrameID("denseVO");

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



/// @brief Prints nowIndex and LastRef index to scratch image. __DEBUG__
/// @param[in] cleanScratch : false indicate that overwrite board. True indicates that need to clean board
void SolveDVO::printFrameIndex2Scratch(cv::Mat &scratch, long nowIndx, long lastRef, double time4Jacobian, double time4Iteration, bool cleanScratch=true)
{
    if( cleanScratch )
        scratch = cv::Mat::ones(400, 400, CV_8UC1 ) * 255;

    char a[100], b[100], c[100], d[100];
    sprintf( a, "Now     : %d", nowIndx );
    sprintf( b, "LastRef : %d", lastRef );
    sprintf( c, "Jacobian : %lf ms", time4Jacobian*1000 );
    sprintf( d, "Iterations : %lf ms", time4Iteration*1000 );
    ROS_DEBUG( a );
    ROS_DEBUG( b );

    cv::putText( scratch, a, cv::Point(20, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0) );
    cv::putText( scratch, b, cv::Point(10, 40), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0) );

    cv::putText( scratch, "Computation Time", cv::Point(10, 60), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0) );

    cv::putText( scratch, c, cv::Point(10, 80), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0) );
    cv::putText( scratch, d, cv::Point(10, 100), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0) );



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
                float Z = _refDepth(yy,xx);
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
/// @param[in] reprojections : 3xN matrix
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

        epsilon(i) = _nowDist( reprojections(1,i), reprojections(0,i) );
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
void SolveDVO::setRefFrame()
{
    assert( isFrameAvailable );
    assert( rcvd_framemono.size() > 0  && rcvd_depth.size() > 0 );

    isRefFrameAvailable = false;
    im_r = rcvd_framemono;
    dim_r = rcvd_depth;
    isRefFrameAvailable = true;
    isJacobianComputed = false;


    computeDistTransfrmOfRef();

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

    computeDistTransfrmOfNow();
}

/*
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
    Eigen::MatrixXf Gx, Gy; //image gradient
    imageGradient(_ref, Gx, Gy );


    // distance map
    Eigen::MatrixXf _distMap = ref_distance_transform[level];
    Eigen::MatrixXf dGx, dGy; //image gradient
    imageGradient(_distMap, dGx, dGy);



    //int nGoodPts = selectedPts(level, Gx, Gy, refROI);
    int nGoodPts = selectedPts(level, refROI);
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


                Eigen::RowVectorXf J_i = G * A1 * A2;



                // register 3d pts, im intensity, im cords, Jacobian (at this interesting pt) in the frame-of-ref of the reference frame
                spC(0,nPtCount) = X;
                spC(1,nPtCount) = Y;
                spC(2,nPtCount) = Z;

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
*/


void SolveDVO::gaussNewtonIterations(int level, int maxIterations, Eigen::Matrix3f& cR, Eigen::Vector3f& cT,
                                     Eigen::VectorXf& energyAtEachIteration, Eigen::VectorXf& finalEpsilons,
                                     Eigen::MatrixXf& finalReprojections, int& bestEnergyIndex, float& finalVisibleRatio)
{

    assert( level >= 0 && level <= 3 );
    assert( maxIterations > 0 );
    assert( isRefFrameAvailable && isNowFrameAvailable && isCameraIntrinsicsAvailable );

    Eigen::MatrixXf _now = im_n[level];
    Eigen::MatrixXf _nowDist = now_distance_transform[level];

    energyAtEachIteration = Eigen::VectorXf::Zero( maxIterations );

    ROS_INFO("-*-*-*-*- Start of Gauss-Newton Iterations (level=%d) -*-*-*-*-", level );

    ROS_DEBUG_STREAM( "init R :\n"<< cR );
    ROS_DEBUG_STREAM( "init T :\n"<< cT.transpose() );


    float prevTotalEpsilon = 1.0E10;

    float bestTotalEpsilon = 1.0E10;
    float bestRatioVisiblePts = 1.0f;
    Eigen::Matrix3f bestcR = Eigen::Matrix3f::Identity();
    Eigen::Vector3f bestcT = Eigen::Vector3f::Zero();
    int bestItrNumber = -1;
    Eigen::VectorXf bestEpsilon;
    Eigen::MatrixXf bestReprojections;

    float lambda = 3.0E7    ;
    float BETA = 0.5;
    Eigen::VectorXf s = Eigen::VectorXf::Zero(6);
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
        computeJacobianOfNowFrame( level, cR, cT, Jcbian, reprojections );


#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__
        ROS_INFO( "size of reprojection : %d %d", reprojections.rows(), reprojections.cols() );
#endif


        //
        // Get corresponding epsilons
        //       Get distances at points given by `reprojections` 2xN matrix
        Eigen::VectorXf epsilon, weights;
        float ratio_of_visible_pts = getReprojectedEpsilons( level, reprojections, epsilon, weights );
        float currentTotalEpsilon = epsilon.norm(); //epsilon.sum() ;
        energyAtEachIteration[itr] = 10.0*currentTotalEpsilon;
#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__
        ROS_INFO( "#%d : Total Epsilon : %f", itr, currentTotalEpsilon );
#endif

        // store the best epsilon
        if( currentTotalEpsilon < bestTotalEpsilon )
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
        // Make normal equations & Solve them
        Eigen::MatrixXf JTW = Jcbian.transpose(); //J' * W
        for( int i=0 ; i<weights.rows() ; i++ )
            JTW.col(i) *= weights(i);







        //////////////////////////////////////////////////
        //                  STEP PARAMOUNT              //
        //      Input  : Jcbian & JTW, epsilon          //
        //      Output : psi                            //
        //////////////////////////////////////////////////
        if( prevTotalEpsilon < currentTotalEpsilon ){ //divergence
            lambda *= 3.0;
        }
        else
            lambda /= 1.5;


        Eigen::VectorXf g = JTW * epsilon; // subgradient of \Sum{ V^2(.) }
        s = (1-BETA)*g + BETA*s;

        Eigen::VectorXf psi = - 1/lambda* s;


#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__
        //ROS_INFO_STREAM( "psi : [ "<< psi.transpose() << "]\n"<< "|psi| : "<< psi.norm() );
        ROS_INFO_STREAM( "|psi| : "<< psi.norm() );
#endif


        //////////////////////////////////////////////////
        //                 END   PARAMOUNT              //
        //      Input  : Jcbian & JTW, epsilon          //
        //      Output : psi                            //
        //////////////////////////////////////////////////




        //
        // Early Termination ?
//        if( psi.norm() < psiNormTerminationThreshold )
//            break;

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





        Eigen::Matrix3f xRot = Eigen::Matrix3f::Identity();
        Eigen::Vector3f xTrans = Eigen::Vector3f::Zero();

        Sophus::SE3f mat = Sophus::SE3f::exp(psi);
        xRot = mat.rotationMatrix();
        xTrans = mat.translation();

        cT = cR*xTrans + cT;
        cR = cR * xRot;


#ifndef __SHOW_REPROJECTIONS_EACH_ITERATION__
        processResidueHistogram( epsilon, true );
#endif   //__SHOW_REPROJECTIONS_EACH_ITERATION__



#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__
        processResidueHistogram( epsilon, false );
#endif   //__SHOW_REPROJECTIONS_EACH_ITERATION__




#ifdef __SHOW_REPROJECTIONS_EACH_ITERATION__
        //
        // DISPLAY
        //printRT( cR, cT, "Updated cR,cT");
        {
        if( __REPROJECTION_LEVEL == level ){
        Eigen::MatrixXi reprojectedMask = Eigen::MatrixXi::Zero(_now.rows(), _now.cols());
        cordList_2_mask(reprojections, reprojectedMask);
        cv::Mat outIm, outImGray, outRef;
        sOverlay(_nowDist, reprojectedMask, outIm, cv::Vec3b(0,255,0));
        sOverlay(_now, reprojectedMask, outImGray, cv::Vec3b(0,255,0));

        sOverlay(im_r[level], ref_edge_map[level], outRef, cv::Vec3b(0,0,255));
        cv::imshow( "reprojection with cR,cT (on now dist-tran", outIm);
        cv::imshow( "reprojection with cR,cT (on now gray", outImGray);
        cv::imshow( "selected edges on ref", outRef);
        visualizeDistanceResidueHeatMap(im_n[level], reprojectedMask, now_distance_transform[level] );



        visualizeEnergyProgress( energyAtEachIteration, bestItrNumber, (energyAtEachIteration.rows() < 300)?4:2 );


        char ch = cv::waitKey(0);
        if( ch == 27 ){ // ESC
            ROS_ERROR( "ESC pressed quitting...");
            exit(1);
        }
        if( ch == 'b')
            break;
        }
        }
        //
        // END DISPLAY
#endif


    }



    // Set : f^* = min{ f_best^(k-1), f^k }
    cR = bestcR;
    cT = bestcT;
    finalEpsilons = bestEpsilon;
    finalReprojections = bestReprojections;
    bestEnergyIndex = bestItrNumber;
    finalVisibleRatio = bestRatioVisiblePts;



    ROS_INFO( "Best Energy Achieved (%f) in iteration #%d with visibility-ratio of %2.2f", bestTotalEpsilon, bestItrNumber, finalVisibleRatio );
    ROS_DEBUG_STREAM( "final R :\n"<< cR );
    ROS_DEBUG_STREAM( "final T :\n"<< cT.transpose() );
    ROS_INFO("-*-*-*-*- End of Gauss-Newton Iterations (level=%d) -*-*-*-*- ", level );



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
//    ///// DISTANCE TRANSFORM of _now
//    ROS_INFO( "distance transform");
//    cv::Mat nownow, xnow, dTrn;
//    cv::eigen2cv( _now, nownow );
//    cv::Sobel( nownow, xnow, CV_8U, 1, 1 );
//    xnow = 255 - xnow;
//    cv::threshold( xnow, xnow, 250, 255, cv::THRESH_BINARY );
//    cv::distanceTransform( xnow, dTrn, CV_DIST_L1, 5 );
//    cv::normalize(dTrn, dTrn, 0.0, 1.0, cv::NORM_MINMAX);
//    cv::imshow("edgeMap", xnow );
//    cv::imshow("distanceTransform", dTrn );

//    double min, max;
//    cv::minMaxLoc(dTrn, &min, &max);
//    ROS_INFO_STREAM( "min : "<< min << " max : "<< max << " dataTYpe : "<< cvMatType2str(dTrn.type()) );
//    ///// END DISTANCE TRANSFORM

    A = Eigen::MatrixXf::Zero(6,6);
    b = Eigen::VectorXf::Zero(6);

    Eigen::MatrixXf cTRep;
    igl::repmat(cT,1,pts3d_ref.cols(),cTRep); // repeat cT col-wise (uses IGL library)
    Eigen::MatrixXf pts3d_n = cR.transpose() * ( pts3d_ref - cTRep );


#if defined(__SHOW_REPROJECTIONS_EACH_ITERATION__) || defined( __COLLECT_EPSILON_DEBUG__DATA_)
    Eigen::MatrixXf pts3d_n_copy = pts3d_n; //note that this is a deep-copy

    if( level == __REPROJECTION_LEVEL )
    {
        __now_roi_reproj = Eigen::MatrixXi::Zero(_now.rows(), _now.cols()); //init to zero
        __residues = -Eigen::VectorXf::Ones( pts3d_ref.cols() ); // init to -1
        __now_roi_reproj_values = -Eigen::MatrixXf::Ones(_now.rows(), _now.cols()); //init to -1
        __reprojected_depth = -Eigen::MatrixXf::Ones( _now.rows(), _now.cols() );
        __weights = -Eigen::MatrixXf::Ones(_now.rows(), _now.cols()); //init to -1

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
    int nSmalkDepthPoint = 0;
    for( int i=0 ; i<pts3d_un_normalized.cols() ; i++ )
    {
        int xx = (int)pts3d_un_normalized(0,i);
        int yy = (int)pts3d_un_normalized(1,i);


        float r=0;


        if( xx >=0 && xx<_now.cols() && yy>= 0 && yy <_now.rows() )
        {


            r = grays_ref(i) - _now(yy,xx);

            float weight = getWeightOf( r );
            if( pts3d_ref(2,i) < 10.0f ) // if the Z of a point is small ignore that point. Smaller Z (usually less than 400) are inaccurate
                weight = 0.0;



            A += (jacob[i].transpose() * jacob[i])*weight;


#if defined(__SHOW_REPROJECTIONS_EACH_ITERATION__) || defined( __COLLECT_EPSILON_DEBUG__DATA_)
            if( level == __REPROJECTION_LEVEL )
            {
#if defined(_IGNORE__NEAR_PTS_DISPLAY____)
                if( pts3d_ref(2,i) >= 100.0f )
#endif
                {
                    __now_roi_reproj(yy,xx) = 1;
                    __now_roi_reproj_values(yy,xx) = (r>0)?r:-r;  // ===> absolute value of r
                    __residues(i) = (r>0)?r:-r;
                    __reprojected_depth(yy,xx) = pts3d_n_copy(2, i );
                    __weights(yy,xx) = weight;
                }

            }
#endif //__SHOW_REPROJECTIONS_EACH_ITERATION__

//            //distance transform based penalty
//            float proximityTerm = 200.0 * dTrn.at<float>(yy,xx);
//            b += (r-proximityTerm)*jacob[i].transpose()*weight;


            b += (r)*jacob[i].transpose()*weight;

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
    return 5.0 / (6.0 + r*r/.25 );
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
/// Given the Gx, Gy (Gradients), selects the interest points based on im grad.
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
            if( _refEdge(yy,xx) > 0 && _refDepth(yy,xx) > 100.0f )
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

}

void SolveDVO::visualizeEnergyProgress(Eigen::VectorXf energyAtEachIteration, int bestEnergyIndex, int XSPACING )
{
    assert( energyAtEachIteration.rows() > 0 );

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
    for( float p=0.0 ; p<100000.0 ; p+=5000.0 ) {
        char str[40];
        sprintf( str, "%2.0fK", (int) p/1000.0 );
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

        cv::distanceTransform( refEdge, refDistTransCvMat, CV_DIST_L1, 5 );
        cv::normalize(refDistTransCvMat, refDistTransCvMat, 0.0, 255.0, cv::NORM_MINMAX);

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

        cv::distanceTransform( nowEdge, nowDistTransCvMat, CV_DIST_L1, 5 );
        cv::normalize(nowDistTransCvMat, nowDistTransCvMat, 0.0, 255.0, cv::NORM_MINMAX);

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


/// @brief The event loop. Basically an ever running while with ros::spinOnce()
/// This is a re-implementation taking into care the memory scopes and also processing only points with higher image gradients
void SolveDVO::loop()
{

    /*
      // Dry Loop
    long nFrame = 0;
    ros::Rate rate(30);
    while( ros::ok() )
    {
        ros::spinOnce();
        if( !(this->isFrameAvailable) )
            continue;

        imshowEigenImage( "current Frame", rcvd_framemono[0] );
        cv::waitKey(1);
//        char ch = cv::waitKey(__ENABLE_DISPLAY__);
//        if( ch == 27 ){ // ESC
//            ROS_ERROR( "ESC pressed quitting...");
//            exit(1);
//        }
            rate.sleep();

    }
    */


    long nFrame=0;
    long lastRefFrame=0;
    double lastRefFrameComputeTime;
    cv::Mat debugScratchBoard = cv::Mat::ones(400, 400, CV_8UC1 ) * 255;
    double gaussNewtonIterationsComputeTime;



    // Pose of now frame wrt currently set reference frame
    Eigen::Matrix3f cR = Eigen::Matrix3f::Identity();
    Eigen::Vector3f cT = Eigen::Vector3f::Zero();

    // Pose of currently set Reference-frame (key frame)
    Eigen::Matrix3f keyR = Eigen::Matrix3f::Identity();
    Eigen::Vector3f keyT = Eigen::Vector3f::Zero();

    // Pose of now frame in global frame of reference
    Eigen::Matrix3f nR = Eigen::Matrix3f::Identity();
    Eigen::Vector3f nT = Eigen::Vector3f::Zero();


    // Additional data returned by Gauss-Newton iterations
    Eigen::VectorXf epsilonVec;
    Eigen::MatrixXf reprojections;
    Eigen::VectorXf energyAtEachIteration;
    int bestEnergyIndex = -1;
    float visibleRatio = 0.0;


    ros::Rate rate(30);
    while( ros::ok() )
    {
        ros::spinOnce();
        if( !(this->isFrameAvailable) )
            continue;

        ROS_INFO( "=-=-=-=-=-=-== Frame # %ld ==-=-=-=-=-=-=", nFrame );


        //if( (nFrame % 9000000) == 0 )
        if( signalGetNewRefImage == true )
        {

            if( nFrame > 0 ) {
                // before changing the reference-frame estimate itz pose
                gaussNewtonIterations(2, 20, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio  );
                gaussNewtonIterations(1, 20, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio );
                gaussNewtonIterations(0, 100, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio );
            }


            lastRefFrame = nFrame;
            setRefFrame();


            ros::Time jstart = ros::Time::now();
            preProcessRefFrame();
            ros::Duration jdur = ros::Time::now() - jstart;
            lastRefFrameComputeTime = jdur.toSec();


            keyR = nR;
            keyT = nT;
            cR = Eigen::Matrix3f::Identity();
            cT = Eigen::Vector3f::Zero();
            signalGetNewRefImage = false;
            sprintf( signalGetNewRefImageMsg, "" );

        }


        setNowFrame();
        ros::Time jstart = ros::Time::now();
        gaussNewtonIterations(2, 7, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio  );
        gaussNewtonIterations(1, 25, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio );
        gaussNewtonIterations(0, 100, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio );
        ros::Duration jdur = ros::Time::now() - jstart;
        gaussNewtonIterationsComputeTime = jdur.toSec();
        ROS_INFO( "Iterations done in %lf ms", gaussNewtonIterationsComputeTime*1000 );




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
        //Debugging display of re-projected points
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


            printFrameIndex2Scratch(debugScratchBoard, nFrame, lastRefFrame, lastRefFrameComputeTime, gaussNewtonIterationsComputeTime, true );
            cv::imshow( "scratBoard", debugScratchBoard );

            visualizeEnergyProgress( energyAtEachIteration, bestEnergyIndex, (energyAtEachIteration.rows() < 300)?4:2 );



            char ch = cv::waitKey(__ENABLE_DISPLAY__);
            if( ch == 27 ){ // ESC
                ROS_ERROR( "ESC pressed quitting...");
                exit(1);
            }
        }
#endif


#ifdef __MINIMAL_DISPLAY
        int xlevel = 0;
        Eigen::MatrixXi reprojectedMask = Eigen::MatrixXi::Zero(im_n[xlevel].rows(), im_n[xlevel].cols());
        cordList_2_mask(reprojections, reprojectedMask);

        cv::Mat outRef;
        sOverlay(im_r[xlevel], ref_edge_map[xlevel], outRef, cv::Vec3b(0,0,255));
        cv::imshow( "selected edges on ref", outRef);


        visualizeDistanceResidueHeatMap(im_n[xlevel], reprojectedMask, now_distance_transform[xlevel] );
        char ch = cv::waitKey(1);
        if( ch == 27 ){ // ESC
            ROS_ERROR( "ESC pressed quitting...");
            exit(1);
        }

#endif //__MINIMAL_DISPLAY


        mviz.publishPoseFinal(nR, nT);
        mviz.publishPath();

        isFrameAvailable = false; //this is put in place to process at-max the arrival rate.

        rate.sleep();
        nFrame++;
    }


}



/// @brief The event loop. Basically an ever running while with ros::spinOnce()
/// This is a re-implementation taking into care the memory scopes and also processing only points with higher image gradients
void SolveDVO::loopFromFile()
{


//    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
//        ros::console::notifyLoggerLevelsChanged();

    char frameFileName[50];
    //const char * folder = "xdump_right2left"; //hard dataset
    //const char * folder = "xdump_left2right";
    const char * folder = "TUM_RGBD/fr1_rpy";

    const int START = 0;
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
    for( int iFrameNum = START; iFrameNum < END ; iFrameNum+=2 )
    {

        sprintf( frameFileName, "%s/framemono_%04d.xml", folder, iFrameNum );
        bool flag = loadFromFile(frameFileName );
        if( flag == false ) {
            ROS_ERROR( "No More files, Quitting..");
            break;
        }


        if( signalGetNewRefImage == true )
        {
            ROS_INFO( "!! NEW REFERENCE FRAME !!");
            ROS_INFO( "Reason : %s", signalGetNewRefImageMsg );

            if( iFrameNum > START ) {
            // before changing the reference-frame estimate itz pose
//            gaussNewtonIterations(2, 20, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio  );
//            gaussNewtonIterations(1, 20, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio );
            gaussNewtonIterations(0, 100, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio );
            }

            keyR = nR;
            keyT = nT;

            lastRefFrame = iFrameNum;
            setRefFrame();

            preProcessRefFrame();

            cR = Eigen::Matrix3f::Identity();
            cT = Eigen::Vector3f::Zero();


            signalGetNewRefImage = false;

        }
        //else
        //{
            setNowFrame();

            ros::Time jstart = ros::Time::now();
            //gaussNewtonIterations(3, 7, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio );
            //gaussNewtonIterations(2, 20, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio  );
//            gaussNewtonIterations(1, 20, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio );
            gaussNewtonIterations(0, 200, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio );
//            gaussNewtonIterations(0, 1300, cR, cT, energyAtEachIteration, epsilonVec, reprojections, bestEnergyIndex, visibleRatio );
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

            visualizeEnergyProgress( energyAtEachIteration, bestEnergyIndex, (energyAtEachIteration.rows() < 300)?4:2 );


            processResidueHistogram( epsilonVec, false );
            if( iFrameNum == START ) { //only attempt to move windows in 1st iteration
            cv::moveWindow("reprojection with cR,cT (on now dist-tran", 800, 600 );
            cv::moveWindow("reprojection with cR,cT (on now gray"     , 1150, 600 );
            cv::moveWindow("selected edges on ref", 1500, 600 );
            }

            ROS_ERROR( "End of Iterations Display.....!");
            }

            //
            // END DISPLAY

#endif


#ifdef __MINIMAL_DISPLAY
        int xlevel = 0;
        Eigen::MatrixXi reprojectedMask = Eigen::MatrixXi::Zero(im_n[xlevel].rows(), im_n[xlevel].cols());
        cordList_2_mask(reprojections, reprojectedMask);

        cv::Mat outRef;
        sOverlay(im_r[xlevel], ref_edge_map[xlevel], outRef, cv::Vec3b(0,0,255));
        cv::imshow( "selected edges on ref", outRef);


        visualizeDistanceResidueHeatMap(im_n[xlevel], reprojectedMask, now_distance_transform[xlevel] );
        char ch = cv::waitKey(1);
        if( ch == 27 ){ // ESC
            ROS_ERROR( "ESC pressed quitting...");
            exit(1);
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
                exit(1);
            }

#endif //__ENABLE_DISPLAY__

    }

}



/*
 * EPoseEstimator.cpp
 *
 *  Created on: 27 Mar, 2015
 *      Author: Manohar Kuse
 * 		 Email: mpkuse@ust.hk
 */

#include "EPoseEstimator.h"

EPoseEstimator::EPoseEstimator() {
	// TODO Auto-generated constructor stub
	isRefFrameAvailable = false;
	isNowFrameAvailable = false;

	isPydImageAvailableRef = false;
	isPydImageAvailableNow = false;

	pydLevel = -1;
	scaleFactor = 1.0;

	isJEvaluated = false; //Jacobian

	is3dCordsReady = false;

}

EPoseEstimator::~EPoseEstimator() {
	// TODO Auto-generated destructor stub
}


/// Input opencv XML file containing camera internal params and distortion coif.
/// Note: At this moment the distortion coif. are not used.
void EPoseEstimator::setCameraMatrix(char* calibFile)
{
	//
	// Read calibration matrix, distortion coiffs
	//
	cv::FileStorage fs(calibFile, cv::FileStorage::READ);
	if( fs.isOpened() == false )
	{
		cerr<< "[PoseEstimator::setCameraMatrix] Error opening camera "
				"params file : "<< calibFile << endl;
		return;
	}

	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;
	cout<< "Camera Matrix : \n"<<  cameraMatrix << endl;
	cout<< "Distortion Coifs : \n"<< distCoeffs << endl;
	fx = cameraMatrix.at<double>(0,0);
	fy = cameraMatrix.at<double>(1,1);
	cx = cameraMatrix.at<double>(0,2);
	cy = cameraMatrix.at<double>(1,2);

	cameraIntrinsicsReady = true;

}


///
/// \brief Sets the "reference" RGBD frame
///
/// Takes the input RGBD and stores it as a reference frame.
/// @param[in] rgb : 3-channels of input
/// @param[in] depth : 16-bit depth image
void EPoseEstimator::setRefFrame(cv::Mat& rgb, cv::Mat& depth) {
	assert( rgb.channels() == 3 );
	rgb.copyTo(c_ref);
	cv::cvtColor( c_ref, g_ref, CV_BGR2GRAY );

	depth.copyTo(refDepth);
	isRefFrameAvailable = true;
	isPydImageAvailableRef = false;

	isJEvaluated = false;
	is3dCordsReady = false;


	//	Evaluate Jacobian and 3d co-ordinates for each pyramid level
	// 	Also store all levels of  `im_r`, `dim_r`, `im_r_color`
	//	This will be espcially benificial because Jacobian is only dependent on reference frame

	// clearing the pyramid is critical here. ie. whenever we have a new Ref frame we need
	// to clear the pyramid
	pydStore.clearPyramid();
	for( int lvl=0 ; lvl<= 4 ; lvl++ )
	{
		cout << "Eval Jacobian for level#"<<lvl<<endl;
		setRefPyramidalImages(lvl);
		evaluateJacobian();


//		cout << "im size : "<< im_r.rows << "  " << im_r.cols << endl;
//		cout << "J size : "<< J.rows() << "  " << J.cols() << endl;
//		cout << J.topRows(5);

		//note that pyDStore uses a std::vector::push_back(), so effectively lvl is ignored
		// Always put the original 1st followed by nxt level n so on.
		pydStore.addLevel(lvl, im_r_color, im_r, dim_r, X, Y, Z, J, grayVals, redVals, greenVals, blueVals);
	}


//	cv::imshow( "pyd level2", pydStore.im_r.at(2) );
//	cv::imshow( "pyd level1", pydStore.im_r.at(1) );
//	cv::imshow( "pyd level0", pydStore.im_r.at(0) );
}


///
/// \brief Sets the "now" RGBD frame
///
/// Takes the input RGBD and stores it as a "now" frame.
/// @param[in] rgb : 3-channels of input
/// @param[in] depth : 16-bit depth image
void EPoseEstimator::setNowFrame(cv::Mat& rgb, cv::Mat& depth) {
	assert( rgb.channels() == 3 );
	rgb.copyTo(c_now);
	cv::cvtColor( c_now, g_now, CV_BGR2GRAY );

	depth.copyTo(nowDepth);
	isNowFrameAvailable = true;
	isPydImageAvailableNow = false;

}


///
/// \brief Estimate the relative R, T of nowFrame wrt to refFrame at a pydLevel
///
/// @param[in,out] R : Input the initial estimate of rotation, finally it will contain the updated estimate
/// @param[in,out] T : Input the initial estimate of translation, finally it will contain the updated estimate
/// @return : percentage of 3d points visible in this view denoted by R T.
float EPoseEstimator::estimate(Eigen::Matrix3d& R, Eigen::Vector3d& T) {
	assert( cameraIntrinsicsReady );
	assert(isRefFrameAvailable);
	assert(isNowFrameAvailable);

	assert(isPydImageAvailableRef);
	assert(isPydImageAvailableNow);



	// Jacobian is already loaded (hopefully) from the pyramid by `setPyramidalImages()`
	// along with it X,Y,Z, and corresponding gray values are also available


	// A = J' * J. A is already evaluated in `setPyramidalImages()`
	assert(isJEvaluated && "J not evaluated\n");


	// 4x4 transform
	Eigen::Matrix4d Tr;
	Eigen::RowVector3d zer_row = Eigen::RowVector3d::Zero();

	Tr << R, T, zer_row, 1.0;


	Eigen::MatrixXd warpedIm;

	float percentageVisible = 1.0;
	for( int itr = 0 ; itr < 3 ; itr++ )
	{
		cout << "$$$$$ Iteration # "<< itr << endl;

		// Warp ref image using Tr
		int nReporj = warpImage(Tr, warpedIm);
		cout << "#total pixels : "<< im.rows*im.cols << endl;
		cout << "# of reprojections : "<< nReporj << endl;
		percentageVisible = nReporj / ( im.rows*im.cols );

		Eigen::MatrixXd nowIm;
		cv::cv2eigen(im, nowIm );

		Eigen::MatrixXd epsilon2d = warpedIm - nowIm;
		epsilon2d.transposeInPlace();
		Eigen::VectorXd epsilon(Eigen::Map<Eigen::VectorXd>(epsilon2d.data(), epsilon2d.cols()*epsilon2d.rows()));

		cout << " J has nan? : "<< J.hasNaN() << endl;

		Eigen::VectorXd b = J.transpose() * epsilon;

		cout << "A:\n"<< A << endl;
		cout << "b :\n"<< b << endl;

		//Eigen::VectorXd del_psi = A.colPivHouseholderQr().solve(b);
		Eigen::VectorXd del_psi = A.householderQr().solve(b);
		cout << "del_psi :\n"<< del_psi << endl;

		Eigen::Matrix4d outTr;
		exponentialMap(del_psi, outTr);

		//Tr = Tr * outTr.inverse();
		Tr = Tr * outTr;

		cout << "Tr :\n"<< Tr << endl;
	}

	arrayShow("warped", warpedIm);


	R = Tr.topLeftCorner(3,3);
	T = Tr.block(0,3,3,1);

	return percentageVisible;


}


/// Set Pyramidal images
/// \note
/// 0 : base level
/// 4 : 1/16 of original
void EPoseEstimator::setPyramidalImages(int level) {
	assert(isRefFrameAvailable);
	assert(isNowFrameAvailable);

	assert(level >= 0 && level <=4 );

	this->pydLevel = level;
	this->scaleFactor = pow( 2, -level );


	// get from the pyramid for ref image
	pydStore.getLevel(level, im_r_color, im_r, dim_r, X,Y,Z, J, grayVals, redVals, greenVals, blueVals);

	A = J.transpose()*J;


	if( level == 0 )
	{
		// same as the original level
//		c_ref.copyTo(im_r_color);
//		g_ref.copyTo(im_r);
//		refDepth.copyTo(dim_r);

		c_now.copyTo(im_color);
		g_now.copyTo(im);
		nowDepth.copyTo(dim);

	}
	else
	{
		// One of the other level
//		cv::resize(c_ref, im_r_color, cv::Size(), scaleFactor, scaleFactor, cv::INTER_AREA );
//		cv::resize(g_ref, im_r, cv::Size(), scaleFactor, scaleFactor, cv::INTER_AREA );
//		cv::resize(refDepth, dim_r, cv::Size(), scaleFactor, scaleFactor, cv::INTER_AREA );

		cv::resize(c_now, im_color, cv::Size(), scaleFactor, scaleFactor, cv::INTER_AREA );
		cv::resize(g_now, im, cv::Size(), scaleFactor, scaleFactor, cv::INTER_AREA );
		cv::resize(nowDepth, dim, cv::Size(), scaleFactor, scaleFactor, cv::INTER_AREA );
	}

	isPydImageAvailableRef = true;
	isPydImageAvailableNow = true;

	isJEvaluated = true;
}

/// Set Reference Pyramidal images
/// \note
/// 0 : base level
/// 4 : 1/16 of original
void EPoseEstimator::setRefPyramidalImages(int level) {
	assert(isRefFrameAvailable);

	assert(level >= 0 && level <=4 );

	this->pydLevel = level;
	this->scaleFactor = pow( 2, -level );

	if( level == 0 )
	{
		// same as the original level
		c_ref.copyTo(im_r_color);
		g_ref.copyTo(im_r);
		refDepth.copyTo(dim_r);
	}
	else
	{
		// One of the other level
		cv::resize(c_ref, im_r_color, cv::Size(), scaleFactor, scaleFactor, cv::INTER_AREA );
		cv::resize(g_ref, im_r, cv::Size(), scaleFactor, scaleFactor, cv::INTER_AREA );
		cv::resize(refDepth, dim_r, cv::Size(), scaleFactor, scaleFactor, cv::INTER_AREA );
	}

	isPydImageAvailableRef = true;
}


/// Just for debugging, will be removed off later
void EPoseEstimator::xdebug() {
	assert(isPydImageAvailableNow && isPydImageAvailableRef);

	cv::imshow("im_r", im_r);
	cv::imshow("dim_r", dim_r);

	cv::imshow("im", im);
	cv::imshow("dim", dim);

	cout << "xdebug X size :"<< X.rows() << "  "<< X.cols() << endl;
	cout << "xdebug J size :"<< J.rows() << "  "<< J.cols() << endl;

	pydStore.printSize();

//	cv::imshow( "key", g_ref );
//	cv::imshow( "now", g_now );
//
//	cv::imshow( "dep key", refDepth );
//	cv::imshow( "dep now", nowDepth );

}

///
/// \brief Evaluates the Nx6 Jacobian matrix (N = # pixels)
///
///	\note The jacobian matrix is only dependent of the keyframe
void EPoseEstimator::evaluateJacobian() {
	assert(cameraIntrinsicsReady);
	assert(isRefFrameAvailable);
	assert(isPydImageAvailableRef);

	///
	/// Step-1
	/// Image Gradients
	/// del(I)/del(PI)
	///
	cv::Mat kernX = (cv::Mat_<float>(3,3) <<  0, 0,  0,
			0,  -1.0, 1.0,
			0, 0,  0);
	cv::Mat kernY = (cv::Mat_<float>(3,3) <<  0, 0,  0,
			0,  -1.0, 0,
			0, 1.0,  0);

	cv::Mat gx, gy;
	cv::filter2D( im_r, gx,CV_64F,kernX );
	cv::filter2D( im_r, gy,CV_64F,kernY );


	// Convert to Eigen::MatrixXd
	Eigen::MatrixXd tmpegx, tmpegy;
	cv::cv2eigen(gx, tmpegx);
	cv::cv2eigen(gy, tmpegy);

	Eigen::ArrayXXd egx, egy;
	egx = tmpegx.array();
	egy = tmpegy.array();



	///
	///	Step-2
	/// Evaluation of 3d Co-ordinates at each pixel.
	///
	evaluate3d();

	assert( is3dCordsReady && "3d cordinates of the reference frame are not ready" );
	Eigen::ArrayXXd Z_inv, Z2_inv;
	Z_inv = 1.0 / Z;
	Z2_inv = 1.0 / (Z*Z);


	// corresponding gray values & color values
	Eigen::MatrixXd tmpGrays, tmpRed, tmpGreen, tmpBlue;
	cv::cv2eigen(im_r, tmpGrays);
	grayVals = tmpGrays.array();

	vector<cv::Mat> channels;
	cv::split(im_r_color, channels);
	cv::cv2eigen(channels[0], tmpBlue);  blueVals = tmpBlue.array();
	cv::cv2eigen(channels[1], tmpGreen); greenVals = tmpGreen.array();
	cv::cv2eigen(channels[2], tmpRed);   redVals = tmpRed.array();



	///
	/// Step-3
	/// Evaluation of Jacobian at each pixel
	/// \frac{dI_2/d(psi) = dI/d(Pi) * d(Pi)/d(x,y,z) * d(x,y,z)/d(psi)
	/// 1x6			1x2			2x3				3x6
	///

	/// [ gx gy ] * [ fx/Z    0    -fx X/Z^2  ] * [ 1 0 0  0  Z  -Y ]
	///				[  0     fy/Z   -fy Y/Z^2 ]	  [ 0 1 0 -Z  0   X ]
	///											  [ 0 0 1  Y  -X  0 ]

	Eigen::ArrayXXd J1(Z.rows(),Z.cols()), J2(Z.rows(),Z.cols()), J3(Z.rows(),Z.cols()), J4(Z.rows(),Z.cols()), J5(Z.rows(),Z.cols()), J6(Z.rows(),Z.cols());
	J1 = fx*egx*Z_inv;
	J2 = fy*egy*Z_inv;
	J3 = -fy*egy*Y*Z2_inv  - fx*egx*X*Z2_inv;
	J4 = egy*( -fy*Y*Y*Z2_inv - fy )  - fx*egx*X*Y*Z2_inv;
	J5 = egx*(fx*X*X*Z2_inv + fx) + fy*egy*X*Y*Z2_inv;
	J6 = fy*egy*X*Z_inv - fx*egy*Y*Z_inv;

	// Convert J1, J2,...,J6 to col matrix (row-major)
//	J1.transposeInPlace();
//	J2.transposeInPlace();
//	J3.transposeInPlace();
//	J4.transposeInPlace();
//	J5.transposeInPlace();
//	J6.transposeInPlace();

	Eigen::VectorXd pJ1(Eigen::Map<Eigen::VectorXd>(J1.data(), J1.cols()*J1.rows()));
	Eigen::VectorXd pJ2(Eigen::Map<Eigen::VectorXd>(J2.data(), J1.cols()*J1.rows()));
	Eigen::VectorXd pJ3(Eigen::Map<Eigen::VectorXd>(J3.data(), J1.cols()*J1.rows()));
	Eigen::VectorXd pJ4(Eigen::Map<Eigen::VectorXd>(J4.data(), J1.cols()*J1.rows()));
	Eigen::VectorXd pJ5(Eigen::Map<Eigen::VectorXd>(J5.data(), J1.cols()*J1.rows()));
	Eigen::VectorXd pJ6(Eigen::Map<Eigen::VectorXd>(J6.data(), J1.cols()*J1.rows()));


	// concat pJ1, PJ2, ..., pJ6 into the matrix J
	Eigen::MatrixXd Jx( J1.cols()*J1.rows(), 6 );
	Jx << pJ1, pJ2, pJ3, pJ4, pJ4, pJ6;

	J = Jx;
//	cout << "egx size : "<< egx.rows() << "  "<< egx.cols() << endl;
//	cout << "Z_inv size : "<< Z_inv.rows() << "  "<< Z_inv.cols() << endl;
//	cout << " egx has nan? : "<< egx.hasNaN() << endl;
//	cout << " Z_inv has nan? : "<< Z_inv.hasNaN() << endl;
//	cout << " J has nan? : "<< J1.hasNaN() << endl;
//
//	cout << "egx.topLeft :\n" << egx.topLeftCorner(5,5) << endl;
//	cout << "Z_inv.topLeft :\n" << Z_inv.topLeftCorner(5,5) << endl;
//	cout << "J1.topLeft :\n" << J1.topLeftCorner(5,5) << endl;


	isJEvaluated = true; // note that this is set to false in `constructor()` and `setRefFrame()`
}

/// \brief Evaluates the 3d co-ordinates from RGBD frame assuming a pin-hole camera model
///
/// Uses the reference RGB & depth. Also uses camera matrix. The results are written into
/// `X`,`Y`,`Z`, `grayVals`, `redVals`, `greenVals`, `blueVals`. They are private variables
/// of the class
///
/// Also note that X,Y,Z are in meter scale
void EPoseEstimator::evaluate3d() {
	assert(cameraIntrinsicsReady && "Camera matrix is not ready");
	assert(isPydImageAvailableRef && "reference pyramid not available");


	//Eigen::MatrixXd X,Y,Z;
	//Eigen::MatrixXd Z_inv, Z2_inv;
	Eigen::MatrixXd tmpZ;
	cv::cv2eigen(dim_r, tmpZ);


	// X,Y,Z are class variables
	Z = tmpZ.array();


	const int xrows = im_r.rows;
	const int xcols = im_r.cols;
	Eigen::VectorXd seqr = Eigen::VectorXd::LinSpaced(xrows,0,xrows-1);
	Eigen::VectorXd seqc = Eigen::VectorXd::LinSpaced(xcols,0,xcols-1);

	//Eigen::MatrixXd U, V;
	Eigen::ArrayXXd U, V;
	U = seqr.replicate(1,xcols); //each element of U is the rowNumber
	V = seqc.replicate(1,xrows).transpose(); //each element of V is the colNumber



	X = Z/fx  *  ( U - scaleFactor*cx );
	Y = Z/fy  *  ( V - scaleFactor*cy );

	// Converting to meter scale to improve on the condition number of J'*J
	X = X/1000.;
	Y = Y/1000.;
	Z = Z/1000.;


	is3dCordsReady = true;

}

/// \brief Warps the reference image using R, T
///
/// @param[in] Tr : 4x4 transformation matrix
/// @return : number of 3d points reprojected in this frame
///
/// \note
/// Uses the RGB & depth of the reference image. It is converted to 3d points using the camera matrix
/// (actually this conversion is already done when ref image is loaded). All these 3d points
/// are changed to new co-ordinate frame, ie. co-ordinate system defined by R|T. In other words,
/// all the 3d points are transformed with inverse( [R|T] ). Then re-projected on the image
/// using the inverse of cameramatrix
int EPoseEstimator::warpImage(Eigen::Matrix4d Tr, Eigen::MatrixXd& canvas) {

	assert(is3dCordsReady && "3d cordinates are not available");



	// X Y Z to col-vector
	Eigen::VectorXd myX(Eigen::Map<Eigen::VectorXd>(X.data(), X.cols()*X.rows()));
	Eigen::VectorXd myY(Eigen::Map<Eigen::VectorXd>(Y.data(), Y.cols()*Y.rows()));
	Eigen::VectorXd myZ(Eigen::Map<Eigen::VectorXd>(Z.data(), Z.cols()*Z.rows()));

	Eigen::VectorXd myGray(Eigen::Map<Eigen::VectorXd>(grayVals.data(), grayVals.cols()*grayVals.rows()));


	Eigen::VectorXd one = Eigen::VectorXd::Ones(X.cols()*X.rows(),1);

	Eigen::MatrixXd P(myX.rows(),4);
	P << myX, myY, myZ, one;


	// Make a 4x4 transformation matrix (inverse for for change of frame)
	Eigen::Matrix4d Tr_inv;

	Tr_inv = Tr.inverse();


	// Change of co-ordinates
	Eigen::MatrixXd P_dash = Tr_inv * P.transpose();



	// Projection
	Eigen::MatrixXd px = P_dash.row(0);
	Eigen::MatrixXd py = P_dash.row(1);
	Eigen::MatrixXd pz = P_dash.row(2);

	Eigen::ArrayXXd u = fx * px.array() / pz.array() + scaleFactor*cx;
	Eigen::ArrayXXd v = fy * py.array() / pz.array() + scaleFactor*cy;


	int nReprojected = 0;

	//loop thru each u,v and write canvas
	canvas = Eigen::MatrixXd::Zero(im_r.rows, im_r.cols);
	for( int i=0 ; i< u.cols() ; i++ )
	{
		int tU = (int)floor(u(i));
		int tV = (int)floor(v(i));
		if( tU >= 0 && tU<im_r.rows-1 && tV >= 0 && tV < im_r.cols-1 )
		{
			nReprojected++;
			canvas( tU, tV)  = myGray( i );
			canvas( tU+1, tV)  = myGray( i );
			canvas( tU, tV+1)  = myGray( i );
			canvas( tU+1, tV+1)  = myGray( i );
		}
	}


	return nReprojected;



}

void EPoseEstimator::arrayShow(char* winName, Eigen::MatrixXd& im) {

	assert(winName != NULL );

	cv::Mat cvCanvas,cvCanvas2;
	cv::eigen2cv(im,cvCanvas);
	cvCanvas.convertTo(cvCanvas2, CV_8UC1 );

	cv::imshow( winName, cvCanvas2 );
}

/// \brief Given a 6-DOF psi, does an exponential Map.
///
/// @param[in] w: Psi 6-vector
/// @param[out] : output transformation matrix
void EPoseEstimator::exponentialMap(Eigen::VectorXd& psi, Eigen::Matrix4d& outTr) {
	assert(psi.rows() == 6 && "PSI does not seem to have 6 rows");

	Eigen::Vector3d t = psi.head(3);
	Eigen::Vector3d w = psi.tail(3);



	Eigen::Matrix3d wx;
	to_se_3(w,wx);


	double theta = w.norm();

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
void EPoseEstimator::to_se_3(Eigen::Vector3d& w, Eigen::Matrix3d& wx) {
	wx = Eigen::Matrix3d::Zero();

	wx(1,2) = -w(0);
	wx(0,2) =  w(1);
	wx(0,1) = -w(2);

	wx(2,1) =  w(0);
	wx(2,0) = -w(1);
	wx(1,0) =  w(2);
}

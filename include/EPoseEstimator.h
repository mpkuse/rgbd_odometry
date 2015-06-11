/*
 * EPoseEstimator.h
 *
 *  Created on: 27 Mar, 2015
 *      Author: Manohar Kuse
 * 		 Email: mpkuse@ust.hk
 */

#ifndef EPOSEESTIMATOR_H_
#define EPOSEESTIMATOR_H_


#include <iostream>
#include <fstream>
#include <vector>
#include <assert.h>
using namespace std;



#include <Eigen/Dense>
#include <Eigen/Geometry>



#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "PyramidalStorage.h"



class EPoseEstimator {
public:
	EPoseEstimator();
	virtual ~EPoseEstimator();

	void setCameraMatrix(char* calibFile);

	void setRefFrame( cv::Mat& rgb, cv::Mat& depth );
	void setNowFrame( cv::Mat& rgb, cv::Mat& depth );


	void setPyramidalImages( int level );
	void setRefPyramidalImages(int level);



	float estimate( Eigen::Matrix3d& initR, Eigen::Vector3d& initT);

	void xdebug();


//private:
	// Camera params
	bool cameraIntrinsicsReady;
	cv::Mat cameraMatrix, distCoeffs;
	double fx, fy, cx, cy;


	// Original Image
	cv::Mat g_ref; ///< Reference (in Gray)
	cv::Mat c_ref; ///< Reference (in color, ie. 3-channel)
	cv::Mat refDepth; ///< Reference depth (16 bits)
	bool isRefFrameAvailable;

	cv::Mat g_now; ///< Now image (in gray)
	cv::Mat c_now; ///< Now image (in color)
	cv::Mat nowDepth; ///< now depth (16 bits)
	bool isNowFrameAvailable;


	// Current RGBD. Written by setPyDImage()
	cv::Mat im_r, dim_r, im_r_color; ///< Images relating to reference image
	cv::Mat im, dim, im_color; ///< Images relating to current image
	bool isPydImageAvailableRef;
	bool isPydImageAvailableNow;
	int pydLevel;
	double scaleFactor;
	PyramidalStorageStruct pydStore;

	//Eigen::MatrixXd J;
	Eigen::MatrixXd J; ///< Jacobian wrt to 6-DOF params
	Eigen::Matrix<double,6,6> A; ///< J' * J. Evaluated on setPyramidalImages()
	bool isJEvaluated;
	void evaluateJacobian();


	// filled in by `evaluateJacobian()`
	Eigen::ArrayXXd X,Y,Z; // XYZ are in meter scale not mm scale
	Eigen::ArrayXXd grayVals,redVals,greenVals,blueVals;
	bool is3dCordsReady;
	void evaluate3d();



	int warpImage( Eigen::Matrix4d Tr, Eigen::MatrixXd& canvas);


	void arrayShow( char * winName, Eigen::MatrixXd& im);


	void exponentialMap( Eigen::VectorXd& psi, Eigen::Matrix4d& outTr );
	void to_se_3( Eigen::Vector3d& w, Eigen::Matrix3d& wx );

};

#endif /* EPOSEESTIMATOR_H_ */

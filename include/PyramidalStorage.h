/*
 * PyramidalStorage.h
 *
 *  Created on: 30 Mar, 2015
 *      Author: Manohar Kuse
 * 		 Email: mpkuse@ust.hk
 */

#ifndef PYRAMIDALSTORAGE_H_
#define PYRAMIDALSTORAGE_H_



#include <iostream>
#include <fstream>
#include <vector>
#include <assert.h>
using namespace std;



#include <Eigen/Dense>
#include <Eigen/Geometry>



#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>




/// Stores the `im_color`, `im_r`, `dim_r`			::> cv::Mat
///	`X`, `Y`, `Z`, 									::> Eigen::ArrayXXd
/// `grayVals`, `redVals`, `greenVals`, `blueVals` 	::> Eigen::ArrayXXd
/// `J`												::> Eigen::Matrix<double,Eigen::Dynamic,6>
class PyramidalStorageStruct {
public:
	PyramidalStorageStruct();
	virtual ~PyramidalStorageStruct();

	void addLevel(int level, cv::Mat& im_r_color, cv::Mat& im_r, cv::Mat& dim_r,
			Eigen::ArrayXXd& X, Eigen::ArrayXXd& Y, Eigen::ArrayXXd& Z,
			Eigen::MatrixXd& J,
			Eigen::ArrayXXd& grayVals,
			Eigen::ArrayXXd& redVals, Eigen::ArrayXXd& greenVals,
			Eigen::ArrayXXd& blueVals);


	void getLevel(int level, cv::Mat& im_r_color,
			cv::Mat& im_r, cv::Mat& dim_r,
			Eigen::ArrayXXd& X, Eigen::ArrayXXd& Y, Eigen::ArrayXXd& Z,
			Eigen::MatrixXd& J,
			Eigen::ArrayXXd& grayVals,
			Eigen::ArrayXXd& redVals, Eigen::ArrayXXd& greenVals,
			Eigen::ArrayXXd& blueVals);

	void clearPyramid();
	void printSize();


private:
	vector<cv::Mat> im_r_color;
	vector<cv::Mat> im_r;
	vector<cv::Mat> dim_r;

	vector<Eigen::ArrayXXd> X;
	vector<Eigen::ArrayXXd> Y;
	vector<Eigen::ArrayXXd> Z;

	vector<Eigen::MatrixXd> J;

	vector<Eigen::ArrayXXd> grayVals;
	vector<Eigen::ArrayXXd> redVals;
	vector<Eigen::ArrayXXd> greenVals;
	vector<Eigen::ArrayXXd> blueVals;

};

#endif /* PYRAMIDALSTORAGE_H_ */

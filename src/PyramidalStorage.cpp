/*
 * PyramidalStorage.cpp
 *
 *  Created on: 30 Mar, 2015
 *      Author: Manohar Kuse
 * 		 Email: mpkuse@ust.hk
 */

#include "PyramidalStorage.h"

PyramidalStorageStruct::PyramidalStorageStruct() {
	// TODO Auto-generated constructor stub
	im_r_color.reserve(5);
	im_r.reserve(5);
	dim_r.reserve(5);

	X.reserve(5);
	Y.reserve(5);
	Z.reserve(5);

	J.reserve(5);

	grayVals.reserve(5);

	redVals.reserve(5);
	greenVals.reserve(5);
	blueVals.reserve(5);

	cout << "My Capacity : "<< J.capacity() << endl;

}

PyramidalStorageStruct::~PyramidalStorageStruct() {
	// TODO Auto-generated destructor stub
}

// uses .push_back() so pls be careful. Effective level is ignored.
void PyramidalStorageStruct::addLevel(int level, cv::Mat& im_r_color,
		cv::Mat& im_r, cv::Mat& dim_r, Eigen::ArrayXXd& X, Eigen::ArrayXXd& Y,
		Eigen::ArrayXXd& Z, Eigen::MatrixXd& J, Eigen::ArrayXXd& grayVals,
		Eigen::ArrayXXd& redVals, Eigen::ArrayXXd& greenVals,
		Eigen::ArrayXXd& blueVals) {

	assert(level >= 0 && level <=4 );

	(this->im_r_color).push_back(im_r_color);
	(this->im_r).push_back(im_r);
	(this->dim_r).push_back(dim_r);

	(this->X).push_back(X);
	(this->Y).push_back(Y);
	(this->Z).push_back(Z);

	(this->J).push_back(J);

	(this->grayVals).push_back(grayVals);

	(this->redVals).push_back(redVals);
	(this->greenVals).push_back(greenVals);
	(this->blueVals).push_back(blueVals);

	//TODO
	//set check flags for each level

}


/// Returns the approapriate level from the pyramid
/// \note
/// At the moment there are no checks about the availability of asked level of pyramid. Caution suggested
void PyramidalStorageStruct::getLevel(int level, cv::Mat& im_r_color,
		cv::Mat& im_r, cv::Mat& dim_r,
		Eigen::ArrayXXd& X, Eigen::ArrayXXd& Y, Eigen::ArrayXXd& Z,
		Eigen::MatrixXd& J,
		Eigen::ArrayXXd& grayVals,
		Eigen::ArrayXXd& redVals, Eigen::ArrayXXd& greenVals,
		Eigen::ArrayXXd& blueVals) {

	assert(level >= 0 && level <=4 );

	//TODO
	//Put in checks here


	im_r_color = (this->im_r_color).at(level);
	im_r = (this->im_r).at(level);
	dim_r = (this->dim_r).at(level);

	X = (this->X).at(level);
	Y = (this->Y).at(level);
	Z = (this->Z).at(level);

	J = (this->J).at(level);

	grayVals = (this->grayVals).at(level);

	redVals = (this->redVals).at(level);
	greenVals = (this->greenVals).at(level);
	blueVals = (this->blueVals).at(level);


}

/// Clears all the vectors
void PyramidalStorageStruct::clearPyramid() {

	im_r_color.clear();
	im_r.clear();
	dim_r.clear();

	X.clear();
	Y.clear();
	Z.clear();

	J.clear();

	grayVals.clear();

	redVals.clear();
	greenVals.clear();
	blueVals.clear();

}

void PyramidalStorageStruct::printSize() {
	cout<< "current Pyramid size : "<< im_r_color.size() << endl;
}

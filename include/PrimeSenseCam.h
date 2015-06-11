/*
 * PrimeSenseCam.h
 *
 *  Created on: 20 Feb, 2015
 *      Author: eeuser
 *
 *      Abstraction layer for Asus Xtion Pro using OpenNI
 *      	Code written using : http://com.occipital.openni.s3.amazonaws.com/OpenNI_Programmers_Guide.pdf
 *	 						& OpenNI Reference
 */

#ifndef PRIMESENSECAM_H_
#define PRIMESENSECAM_H_

// STandard Headers
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
using namespace std;

// OpenNI
#include <openni2/OpenNI.h>


// OpenCV
#include <opencv2/opencv.hpp>


class PrimeSenseCam {
public:
	PrimeSenseCam();
	virtual ~PrimeSenseCam();


	void start();
	void retriveFrame( cv::Mat& pFrame, cv::Mat& dFrame );

	int getdDataSize() const {
		return dDataSize;
	}

	int getdHeight() const {
		return dHeight;
	}

	int getdWidth() const {
		return dWidth;
	}

	int getpDataSize() const {
		return pDataSize;
	}

	int getpHeight() const {
		return pHeight;
	}

	int getpWidth() const {
		return pWidth;
	}

private:
	openni::Device device;
	openni::VideoStream stream;
	openni::VideoStream depthStream;

	openni::VideoFrameRef * pframe; //color frame
	openni::VideoFrameRef * dframe; // depth frame

	void setDeviceParams();
	void setStreamParams();

	void retriveFrame( uchar *f, cv::Mat imx );


	int pWidth;
	int pHeight;
	int pDataSize;

	int dWidth;
	int dHeight;
	int dDataSize;
};

#endif /* PRIMESENSECAM_H_ */

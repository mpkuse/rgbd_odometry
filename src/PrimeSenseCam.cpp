/*
 * PrimeSenseCam.cpp
 *
 *  Created on: 20 Feb, 2015
 *      Author: eeuser
 */

#include <PrimeSenseCam.h>

PrimeSenseCam::PrimeSenseCam() {

	// Note, setting tuned for Asus Xtion Pro Live.
	// In case another cam please refer to OpenNI programming guide and examples

	if( openni::OpenNI::initialize() != openni::STATUS_OK )
	{
		cerr << "Fail\n";
		exit(1);
	}


	// Opening Any available device
	const char * deviceuri = openni::ANY_DEVICE;
	if( device.open(deviceuri) != openni::STATUS_OK )
	{
		cout<< "Device Opening failed\n";
		exit(1);
	}
	cout<< "Device Vendor : "<< device.getDeviceInfo().getVendor() << endl;
	cout<< "Device Name : "<< device.getDeviceInfo().getName() << endl;
	if( device.hasSensor(openni::SENSOR_COLOR) )
		cout<< "This device has a Color Sensor\n";

	if( device.hasSensor(openni::SENSOR_DEPTH) )
		cout<< "This device has a Depth Sensor\n";

	if( device.hasSensor(openni::SENSOR_IR) )
		cout<< "This device has an IR Sensor\n";

	setDeviceParams();

	// Opening video stream
	if( stream.create( device, openni::SENSOR_COLOR ) != openni::STATUS_OK )
	{
		cerr << "Failed to create video stream\n";
		exit(1);
	}
	if( depthStream.create( device, openni::SENSOR_DEPTH ) != openni::STATUS_OK )
	{
		cerr << "Failed to depth create video stream\n";
		exit(1);
	}

	setStreamParams();






	pframe =  new openni::VideoFrameRef();
	dframe = new openni::VideoFrameRef();

	//start stream
	start();

}

PrimeSenseCam::~PrimeSenseCam() {
	delete pframe;
	delete dframe;
	stream.stop();
	depthStream.stop();
	stream.destroy();
	depthStream.destroy();
	device.close();
	//openni::OpenNI::shutdown();
}

// sets device params like mirroring, sync enable, etc
void PrimeSenseCam::setDeviceParams() {
	//enable framesync
	cout << "Setting Depth-Color Sync\n";
	device.setDepthColorSyncEnabled(true);

	//device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);
	if( device.isImageRegistrationModeSupported(device.getImageRegistrationMode()) )
		cout<< "Depth-Color Registration is supported\n";

}


// start streaming data
void PrimeSenseCam::start() {
	// start streams
	depthStream.start();
	stream.start();

	// load one frame to test
	if( depthStream.readFrame(dframe) == openni::STATUS_OK )
		cout<< "Frame depth read success\n";

	if( stream.readFrame(pframe) == openni::STATUS_OK )
		cout<< "Frame read success\n";

	pWidth = pframe->getWidth();
	pHeight = pframe->getHeight();
	pDataSize = pframe->getDataSize();

	dWidth = dframe->getWidth();
	dHeight = dframe->getHeight();
	dDataSize = dframe->getDataSize();
}

void PrimeSenseCam::retriveFrame(cv::Mat& pIm, cv::Mat& dIm) {
	// check if memory is allocated
	if( pIm.rows != pHeight || pIm.cols != pWidth || pIm.type() != CV_8UC3 )
	{
		pIm = cv::Mat::zeros(pframe->getHeight(), pframe->getWidth(), CV_8UC3);
	}


	// Retrive frame from CAM
	if( depthStream.readFrame(dframe) != openni::STATUS_OK )
		cerr<< "Error Reading depth frame\n";

	if( stream.readFrame(pframe) != openni::STATUS_OK )
		cerr<< "Error Reading frame\n";


	unsigned char * colorRawData= (unsigned char*)pframe->getData();
	retriveFrame( colorRawData, pIm );


	openni::DepthPixel * d = (openni::DepthPixel *)dframe->getData();
	dIm = cv::Mat( dframe->getHeight(), dframe->getWidth(), CV_16S, dframe->getData() );


}

void PrimeSenseCam::setStreamParams() {
    cout << "Disable Mirroring\n";
    depthStream.setMirroringEnabled(false);
    stream.setMirroringEnabled(false);


	// setting resolution, FPS, pixel-format
	openni::VideoMode vMode;
	vMode.setFps(30);
	vMode.setResolution(640,480);
	vMode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
	openni::Status vmodeColSta = stream.setVideoMode(vMode);
	vMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
	openni::Status vmodeDepSta = depthStream.setVideoMode(vMode);
	if ( (vmodeColSta != openni::STATUS_OK) || (vmodeDepSta != openni::STATUS_OK))
	{
		cout << "[setStreamParams] error: Video format not supported..." << endl;
		exit(1);
	}


	// auto exposure & white balance
	openni::CameraSettings * camsettings = stream.getCameraSettings();
	camsettings->setAutoExposureEnabled(false);
	camsettings->setAutoWhiteBalanceEnabled(false);
}


void PrimeSenseCam::retriveFrame( uchar *f, cv::Mat imx )
{
	int height = imx.rows;
	int width = imx.cols;
	int widthStep=0;
	for( int h=0 ; h<height ; h++ )
	{
		for( int w=0 ; w<width ; w++ )
		{
			imx.at<cv::Vec3b>(h,w)[0] = (uchar)f[ widthStep + 3*w + 2];
			imx.at<cv::Vec3b>(h,w)[1] = (uchar)f[ widthStep + 3*w + 1];
			imx.at<cv::Vec3b>(h,w)[2] = (uchar)f[ widthStep + 3*w ];
		}
		widthStep += (3*width);
	}
}

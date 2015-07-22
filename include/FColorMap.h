#ifndef ___FColorMap
#define ___FColorMap

/// @file Class defination of a false color map


#include <ros/ros.h>
#include <opencv2/opencv.hpp>


class FColorMap
{
public:
    FColorMap();
    FColorMap(int nColors);
    cv::Vec3b& at( int i );

private:
    std::vector<cv::Vec3b> colors;
};


#endif //___FColorMap

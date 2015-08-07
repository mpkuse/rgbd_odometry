#include <GOP.h>



GOP::GOP()
{
    keyFrame = false;
    frameId = -1;
    reason_of_change = -1;

    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Vector3f O = Eigen::Vector3f::Zero(); // this is 'ooo' not zero :P

    nowRel_R = I;
    world_R = I;
    lastKeyFrame_R = I;

    nowRel_T = O;
    world_T = O;

    lastKeyFrame_R = I;
    lastKeyFrame_T = O;


}

void GOP::setAsOrdinaryFrame(int frameNum, Eigen::Matrix3f cR, Eigen::Vector3f cT)
{
    this->keyFrame = false;
    this->reason_of_change = -1;

    this->frameId = frameNum;

    nowRel_R = cR;
    nowRel_T = cT;
}

void GOP::setAsKeyFrame(int frameNum, int reason, Eigen::Matrix3f cR, Eigen::Vector3f cT)
{
    this->keyFrame = true;
    this->reason_of_change = reason;

    this->frameId = frameNum;

    nowRel_R = cR;
    nowRel_T = cT;
}


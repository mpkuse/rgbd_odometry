#include <GOP.h>

/////////////////////// GOP ELEMENT CLASS DECLARATIONS ////////////////////////

GOPElement::GOPElement()
{
    keyFrame = false;
    frameId = -1;
    reason_of_change = -1;

    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Vector3f O = Eigen::Vector3f::Zero(); // this is 'ooo' not zero :P

    world_R = I;
    world_T = O;
}


/// Set current frame as non-keyframe.
/// @param[in] frameNum : Frame ID
/// @param[in] cR : Global Pose (wrt nFrame=0)
/// @param[in] cT : Global Pose (wrt nFrame=0)
void GOPElement::setAsOrdinaryFrame(int frameNum, Eigen::Matrix3f wR, Eigen::Vector3f wT)
{
    this->keyFrame = false;
    this->reason_of_change = -1;

    this->frameId = frameNum;

    world_R = wR;
    world_T = wT;

    matrixToPose(wR, wT, world_pose );
}


/// Set current frame as keyframe.
/// @param[in] frameNum : Frame ID
/// @param[in] reasonCode : (1) 1st frame. (2) laplacian threshold breached. (3) ratio of visible pt less than a threshold. (4) # of reprojected pts less than 50
/// @param[in] cR : Global Pose (wrt nFrame=0)
/// @param[in] cT : Global Pose (wrt nFrame=0)
void GOPElement::setAsKeyFrame(int frameNum, int reasonCode, Eigen::Matrix3f wR, Eigen::Vector3f wT)
{
    this->keyFrame = true;
    this->reason_of_change = reasonCode;

    this->frameId = frameNum;

    world_R = wR;
    world_T = wT;

    matrixToPose(wR, wT, world_pose );
}

const Eigen::Matrix3f &GOPElement::getR()
{
    return world_R;
}

const Eigen::Vector3f &GOPElement::getT()
{
    return world_T;
}

const geometry_msgs::Pose &GOPElement::getPose()
{
    return world_pose;
}

bool GOPElement::isKeyFrame()
{
    return keyFrame;
}


int GOPElement::getReason()
{
    if( keyFrame )
        return reason_of_change;
    else
        return -1;
}


/// @brief Given the rotation and translation matrix convert to ros Pose representation
/// @param[in] rot : 3x3 rotation matrix
/// @param[in] trans : 3-vector representing translation
/// @param[out] rosPose : geometry_msgs::Pose as output
void GOPElement::matrixToPose(Eigen::Matrix3f rot, Eigen::Vector3f tran, geometry_msgs::Pose& rospose)
{
    Eigen::Quaternionf quat(rot);

    rospose.position.x = tran(0);
    rospose.position.y = tran(1);
    rospose.position.z = tran(2);
    rospose.orientation.x = quat.x();
    rospose.orientation.y = quat.y();
    rospose.orientation.z = quat.z();
    rospose.orientation.w = quat.w();
}


/////////////////////// END GOP ELEMENT CLASS DECLARATIONS ////////////////////////



/// Constructor for GOP class
GOP::GOP()
{
    gopVector.reserve(100000);
    gopVector.clear();

    lastKeyFr_R = Eigen::Matrix3f::Identity();
    lastKeyFr_T = Eigen::Vector3f::Zero();
}


/// Set current frame as non-keyframe.
/// @param[in] frameNum : Frame ID
/// @param[in] cR : Relative pose (wrt last keyframe)
/// @param[in] cT : Relative pose (wrt last keyframe)
void GOP::pushAsOrdinaryFrame(int frameNum, Eigen::Matrix3f cR, Eigen::Vector3f cT)
{
    //
    //make a global pose
    //  [keyR keyT] x [cR cT]
    //  [ 0    1  ] x [0   1]
    Eigen::Vector3f global_T = lastKeyFr_T + lastKeyFr_R*cT;
    Eigen::Matrix3f global_R = lastKeyFr_R*cR;


    // push onto the vector
    GOPElement ele;
    ele.setAsOrdinaryFrame(frameNum, global_R, global_T);

    gopVector.push_back(ele);

}



/// Set current frame as keyframe.
/// @param[in] frameNum : Frame ID
/// @param[in] reasonCode : (1) 1st frame. (2) laplacian threshold breached. (3) ratio of visible pt less than a threshold. (4) # of reprojected pts less than 50
/// @param[in] cR : Relative pose (wrt last keyframe)
/// @param[in] cT : Relative pose (wrt last keyframe)
void GOP::pushAsKeyFrame(int frameNum, int reason, Eigen::Matrix3f cR, Eigen::Vector3f cT)
{
    //
    //make a global pose
    //  [keyR keyT] x [cR cT]
    //  [ 0    1  ] x [0   1]
    Eigen::Vector3f global_T = lastKeyFr_T + lastKeyFr_R*cT;
    Eigen::Matrix3f global_R = lastKeyFr_R*cR;


    //
    // push onto the vector
    GOPElement ele;
    ele.setAsKeyFrame(frameNum, reason, global_R, global_T);

    gopVector.push_back(ele);


    //
    // update this->`lastKeyFrame` pose
    lastKeyFr_R = global_R;
    lastKeyFr_T = global_T;
}

/// Returns size of the gop Vector
int GOP::size()
{
    return (int) gopVector.size();
}

/// Pose at i^{th} pose
const geometry_msgs::Pose & GOP::getGlobalPoseAt(int i )
{
    assert( i < gopVector.size() );
    return gopVector[i].getPose();

}

const Eigen::Matrix3f & GOP::getGlobalRAt(int i )
{
    assert( i < gopVector.size() );
    return gopVector[i].getR();

}

const Eigen::Vector3f & GOP::getGlobalTAt(int i)
{
    assert( i < gopVector.size() );
    return gopVector[i].getT();
}

bool GOP::isKeyFrameAt(int i)
{
    assert( i < gopVector.size() );
    return gopVector[i].isKeyFrame();
}

int GOP::getReasonAt(int i)
{
    assert( i < gopVector.size() );
    return gopVector[i].getReason();
}


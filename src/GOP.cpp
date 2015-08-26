#include <GOP.h>

/////////////////////// GOP ELEMENT CLASS DECLARATIONS ////////////////////////
template <typename T>
GOPElement<T>::GOPElement()
{
    keyFrame = false;
    frameId = -1;
    reason_of_change = -1;

    Eigen::Matrix<T,3,3> I = Eigen::Matrix<T,3,3>::Identity();
    Eigen::Matrix<T,3,1> O = Eigen::Matrix<T,3,1>::Zero(); // this is 'ooo' not zero :P

    world_R = I;
    world_T = O;
}


/// Set current frame as non-keyframe.
/// @param[in] frameNum : Frame ID
/// @param[in] cR : Global Pose (wrt nFrame=0)
/// @param[in] cT : Global Pose (wrt nFrame=0)
template <typename T>
void GOPElement<T>::setAsOrdinaryFrame(int frameNum, Eigen::Matrix<T,3,3> wR, Eigen::Matrix<T,3,1> wT)
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
template <typename T>
void GOPElement<T>::setAsKeyFrame(int frameNum, int reasonCode, Eigen::Matrix<T,3,3> wR, Eigen::Matrix<T,3,1> wT)
{
    this->keyFrame = true;
    this->reason_of_change = reasonCode;

    this->frameId = frameNum;

    world_R = wR;
    world_T = wT;

    matrixToPose(wR, wT, world_pose );
}

template <typename T>
Eigen::Matrix<T,3,3> const &GOPElement<T>::getR()
{
    return world_R;
}

template <typename T>
const Eigen::Matrix<T,3,1> &GOPElement<T>::getT()
{
    return world_T;
}

template <typename T>
const geometry_msgs::Pose &GOPElement<T>::getPose()
{
    return world_pose;
}

template <typename T>
bool GOPElement<T>::isKeyFrame()
{
    return keyFrame;
}

template <typename T>
int GOPElement<T>::getReason()
{
    if( keyFrame )
        return reason_of_change;
    else
        return -1;
}

template <typename T>
void GOPElement<T>::updateAsKeyFrame(int reason)
{
    keyFrame = true;
    reason_of_change = reason;
}


/// @brief Given the rotation and translation matrix convert to ros Pose representation
/// @param[in] rot : 3x3 rotation matrix
/// @param[in] trans : 3-vector representing translation
/// @param[out] rosPose : geometry_msgs::Pose as output
template <typename T>
void GOPElement<T>::matrixToPose(Eigen::Matrix<T,3,3> rot, Eigen::Matrix<T,3,1> tran, geometry_msgs::Pose& rospose)
{
    Eigen::Quaternion<T> quat(rot);

    rospose.position.x = tran(0);
    rospose.position.y = tran(1);
    rospose.position.z = tran(2);
    rospose.orientation.x = quat.x();
    rospose.orientation.y = quat.y();
    rospose.orientation.z = quat.z();
    rospose.orientation.w = quat.w();
}


/////////////////////// END GOP ELEMENT CLASS DEFINATIONS ////////////////////////



/// Constructor for GOP class
template <typename T>
GOP<T>::GOP()
{
    gopVector.reserve(100000);
    gopVector.clear();

    lastKeyFr_R = Eigen::Matrix<T,3,3>::Identity();
    lastKeyFr_T = Eigen::Matrix<T,3,1>::Zero();
}


/// Set current frame as non-keyframe.
/// @param[in] frameNum : Frame ID
/// @param[in] cR : Relative pose (wrt last keyframe)
/// @param[in] cT : Relative pose (wrt last keyframe)
template <typename T>
void GOP<T>::pushAsOrdinaryFrame(int frameNum, Eigen::Matrix<T,3,3> cR, Eigen::Matrix<T,3,1> cT)
{
    //
    //make a global pose
    //  [keyR keyT] x [cR cT]
    //  [ 0    1  ] x [0   1]
    Eigen::Matrix<T,3,1> global_T = lastKeyFr_T + lastKeyFr_R*cT;
    Eigen::Matrix<T,3,3> global_R = lastKeyFr_R*cR;


    // push onto the vector
    GOPElement<T> ele;
    ele.setAsOrdinaryFrame(frameNum, global_R, global_T);

    gopVector.push_back(ele);

}



/// Set current frame as keyframe.
/// @param[in] frameNum : Frame ID
/// @param[in] reasonCode : (1) 1st frame. (2) laplacian threshold breached. (3) ratio of visible pt less than a threshold. (4) # of reprojected pts less than 50
/// @param[in] cR : Relative pose (wrt last keyframe)
/// @param[in] cT : Relative pose (wrt last keyframe)
template <typename T>
void GOP<T>::pushAsKeyFrame(int frameNum, int reason, Eigen::Matrix<T,3,3> cR, Eigen::Matrix<T,3,1> cT)
{
    //
    //make a global pose
    //  [keyR keyT] x [cR cT]
    //  [ 0    1  ] x [0   1]
    Eigen::Matrix<T,3,1> global_T = lastKeyFr_T + lastKeyFr_R*cT;
    Eigen::Matrix<T,3,3> global_R = lastKeyFr_R*cR;


    //
    // push onto the vector
    GOPElement<T> ele;
    ele.setAsKeyFrame(frameNum, reason, global_R, global_T);

    gopVector.push_back(ele);


    //
    // update this->`lastKeyFrame` pose
    lastKeyFr_R = global_R;
    lastKeyFr_T = global_T;
}

template <typename T>
void GOP<T>::updateMostRecentToKeyFrame( int reason )
{
    int mostRecentIndx = gopVector.size() - 1;
    lastKeyFr_R = gopVector[mostRecentIndx].getR();
    lastKeyFr_T = gopVector[mostRecentIndx].getT();

    gopVector[mostRecentIndx].updateAsKeyFrame(reason);
}

/// Returns size of the gop Vector
template <typename T>
int GOP<T>::size()
{
    return (int) gopVector.size();
}

/// Pose at i^{th} pose
template <typename T>
const geometry_msgs::Pose & GOP<T>::getGlobalPoseAt(int i )
{
    assert( i < gopVector.size() );
    return gopVector[i].getPose();

}

template <typename T>
const Eigen::Matrix<T,3,3> & GOP<T>::getGlobalRAt(int i )
{
    assert( i < gopVector.size() );
    return gopVector[i].getR();

}

template <typename T>
const Eigen::Matrix<T,3,1> & GOP<T>::getGlobalTAt(int i)
{
    assert( i < gopVector.size() );
    return gopVector[i].getT();
}

template <typename T>
bool GOP<T>::isKeyFrameAt(int i)
{
    assert( i < gopVector.size() );
    return gopVector[i].isKeyFrame();
}

template <typename T>
int GOP<T>::getReasonAt(int i)
{
    assert( i < gopVector.size() );
    return gopVector[i].getReason();
}


template class GOP<float>;
template class GOP<double>;

/*
 * <one line to give the library's name and an idea of what it does.>
 * Copyright (C) 2013  Kourosh <email>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <common/VisualOdometryInterface.h>

#define DEFAULT_KEYFRAMES_DIST            0.1
#define DEFAULT_KEYFRAMES_ANGLE           20.0
#define DEFAULT_ROBOT_HEIGHT              0.57

namespace KSRobot
{
namespace common
{

VisualOdometryInterface::VisualOdometryInterface(): Interface(),
    mIsCycleKeyframe(true), // first cycle is always keyframe
    mPublishKeyframeDescriptors(true),
    mLastKinectCycle(-1), mOdomTimer(new Timer("Odometry time")),
    mFinishCycleTime(new Timer("FinishCycle time")),
    mSetHeight(false), mRobotHeight(0), mIsEveryCycleKeyframe(false),
    mUseMovementThresholdsForKeyframes(false), mMovementThr(0), mYawThr(0)
{
    mMotion = new MotionInfo;
    mLatestKeyframe.reset(new VisualKeyframe);
    
    RegisterTimer(mOdomTimer);
    RegisterTimer(mFinishCycleTime);
}

VisualOdometryInterface::~VisualOdometryInterface()
{
    delete mMotion;
}

void VisualOdometryInterface::RegisterToKinect(KinectInterface::Ptr ki)
{
    mKinect = ki;
}

bool VisualOdometryInterface::RunSingleCycle()
{
    Interface::ScopedLock lock(this);
    Interface::ScopedLock kinectLock(mKinect.get());
    
    if( mKinect->GetCycle() <= mLastKinectCycle )
        return false;
    
    mCurrFloatDepth = mKinect->GetFloatDepthImage();
    mCurrRawDepth = mKinect->GetRawDepthImage();
    mCurrRgb = mKinect->GetRgbImage();
    mCurrPointCloud = mKinect->GetPointCloud();
    
    mLastKinectCycle = mKinect->GetCycle();
    return true;
}

void VisualOdometryInterface::NotifyKeyframeReceivers()
{
    if( IsThisCycleKeyframe() )
        mKeyframeReceivers(GetLatestKeyframe());
}

void VisualOdometryInterface::FinishCycle()
{
    mFinishCycleTime->Start();
    
    Interface::FinishCycle();
    
    mMotion->GlobalPose = mMotion->GlobalPose * mMotion->MotionEstimate;
    if( mSetHeight )
        mMotion->GlobalPose.translation()[2] = mRobotHeight;
    
    mIsCycleKeyframe = CheckForKeyframe();
    if( IsThisCycleKeyframe() )
    {
        mLatestKeyframe.reset(new VisualKeyframe);
        mLatestKeyframe->GlobalPose = mMotion->GlobalPose;
        mLatestKeyframe->RelativeMotion = mMotion->LastKeyframePose.inverse() * mMotion->GlobalPose;
        
        if( GetCycle() < 2 ) // Cannot have a valid motion estimate before 2nd input, motion is set to identity.
            mLatestKeyframe->MotionEstimateAvailable = false;
        else
            mLatestKeyframe->MotionEstimateAvailable = true;
        
        PublishKeyframeFeatures(mLatestKeyframe);
        
        mMotion->CurrRelativeMotion.setIdentity();
        mMotion->LastKeyframePose = mMotion->GlobalPose;

        NotifyKeyframeReceivers();
    }
    else
    {
        mMotion->CurrRelativeMotion = mMotion->LastKeyframePose.inverse() * mMotion->GlobalPose;
    }
    
    mFinishCycleTime->Stop();
}

bool VisualOdometryInterface::CheckForKeyframe()
{
    if( mIsEveryCycleKeyframe )
    {
//         Debug("(VisualOdometryInterface::CheckForKeyframe) Every cycle is keyframe.\n");
        return true;
    }
    
    if( GetCycle() == 0 )
    {
//         Debug("(VisualOdometryInterface::CheckForKeyframe) cycle is 0.\n");
        return true;
    }
    
    if( mUseMovementThresholdsForKeyframes )
    {
        
        // first check for movement threshold
        if( mMotion->CurrRelativeMotion.translation().squaredNorm() >= mMovementThr * mMovementThr )
            return true;
        //now check for yaw angle.
        //NOTE: SINCE IN FOVIS OR OTHER VO Algorithms, the coordinate is usually in the way that z is front, yaw might not be the actual yaw
        //          calculated here.
        
        //for now just debug
        
        Eigen::Matrix<float,3,1> euler = mMotion->CurrRelativeMotion.rotation().eulerAngles(2, 1, 0);
        //float yaw = euler(0, 0) * 180.0 / M_PI;
        float pitch = euler(1, 0) * 180.0 / M_PI;
        //float roll = euler(2, 0) * 180.0 / M_PI;
        
        //It is correct that in normal world condition we should check the yaw angle, but because of kinect data aquizition, 
        // the yaw angle in robot frame is pitch angle in kinect frame, which we will use here.
        //NOTE: For now I don't really use robot frame, and always use kinect frame.
        
        if( pitch >= mYawThr )
            return true;
    }
    
    return false;
}

void VisualOdometryInterface::ReadSettings(ProgramOptions::Ptr po)
{
    Interface::ReadSettings(po);
    
    mIsEveryCycleKeyframe = po->GetBool("IsEveryCycleKeyframe", false);
    mUseMovementThresholdsForKeyframes = po->GetBool("UseMovementThresholdsForKeyframes", false);
    mMovementThr = po->GetDouble("MovementThr", 0.1);
    mYawThr = po->GetDouble("YawThr", 20.0);
    mPublishKeyframeDescriptors = po->GetBool("PublishKeyframeDescriptors", true);
}

void VisualOdometryInterface::SetRobotInfo(RobotInfo::Ptr roboinfo)
{
    mSetHeight = roboinfo->AddHeightToOdometry();
    mRobotHeight = roboinfo->GetConstantHeight();
}


} // end namespace common
} // end namespace KSRobot

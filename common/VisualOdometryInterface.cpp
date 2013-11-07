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
    mPublishKeyframeDescriptors(false),
    mConverged(false),
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
    //FIXME: For the current tests, we don't run vo in parallel to kinect and call RunSingleCycle inside kinect's OnFinish
    //          this causes the kinect to be already locked.
    //Interface::ScopedLock kinectLock(mKinect.get()); 
    
    if( mKinect->GetCycle() <= mLastKinectCycle )
        return false;
    
    mPrevPointCloud = mCurrPointCloud;
    mPrevRgb = mCurrRgb;
    mPrevGray = mCurrGray;
    mPrevRawDepth = mCurrRawDepth;
    mPrevFloatDepth = mCurrFloatDepth;
    
    mCurrFloatDepth = mKinect->GetFloatDepthImage();
    mCurrRawDepth = mKinect->GetRawDepthImage();
    mCurrRgb = mKinect->GetRgbImage();
    mCurrGray = mKinect->GetGrayImage();
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
    
    //mMotion->MotionEstimate.translation()[1] = 0;
    mMotion->GlobalPose = mMotion->GlobalPose * mMotion->MotionEstimate;
    //if( mSetHeight )
    
    if( mProjectMovementOnGround )
    {
        mMotion->GlobalPose.translation()[mProjectToAxis] = mRobotHeight;
        Eigen::Matrix<float, 3, 1> euler = mMotion->GlobalPose.rotation().eulerAngles(2, 1, 0);
        
        Eigen::Affine3f t = mMotion->GlobalPose;
        
        float roll = atan2f(t(2,1), t(2,2));
        float pitch = asinf(-t(2,0));
        float yaw = atan2f(t(1,0), t(0,0));
        
//         float yaw = euler(0, 0);
//         float pitch = euler(1, 0);
//         float roll = euler(2, 0);
        
        Eigen::Matrix3f Rx, Ry, Rz;
        
        if( mProjectToAxis == 0 )
        {
            float s1 = sin(roll);
            float c1 = cos(roll);
            Rx <<   1,  0,  0,
            0, c1,-s1,
            0, s1, c1;
            mMotion->GlobalPose.linear() = Rx;
        }
        else if( mProjectToAxis == 1 ) // default for kinect
        {
            float s2 = sin(pitch);
            float c2 = cos(pitch);
            Ry <<  c2,  0, s2,
            0,  1,  0,
            -s2,  0, c2;
            mMotion->GlobalPose.linear() = Ry;
        }
        else
        {
            float s3 = sin(yaw);
            float c3 = cos(yaw);
            Rz <<  c3,-s3,  0,
            s3, c3,  0,
            0,  0,  1;
            mMotion->GlobalPose.linear() = Rz;
        }
        //mMotion->GlobalPose.linear() = Rx * Ry * Rz;
    }
        
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
        std::cout << "ALL KEYFRAME\n" << std::flush;
        return true;
    }
    if( GetCycle() == 0 )
        return true;
    
    if( mUseMovementThresholdsForKeyframes )
    {
        // first check for movement threshold
        if( mMotion->CurrRelativeMotion.translation().squaredNorm() >= mMovementThr * mMovementThr )
        {
            return true;
        }
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
        //NOTE: This also assumes small motions.
        if( pitch >= mYawThr && 180 - pitch >= mYawThr ) // to prevent 179.9 degree pitch from trigerring keyframe change
        {
            return true;
        }
    }
    
    return false;
}

void VisualOdometryInterface::ReadSettings(ProgramOptions::Ptr po)
{
    Interface::ReadSettings(po);
    
    mProjectMovementOnGround = po->GetBool("ProjectMovementOnGround", false);
    mProjectToAxis = po->GetAxis("ProjectToAxis", std::string("y"));
    
    mIsEveryCycleKeyframe = po->GetBool("IsEveryCycleKeyframe", false);
    mUseMovementThresholdsForKeyframes = po->GetBool("UseMovementThresholdsForKeyframes", false);
    mMovementThr = po->GetDouble("MovementThr", 0.1);
    mYawThr = po->GetDouble("YawThr", 20.0);
    mPublishKeyframeDescriptors = po->GetBool("PublishKeyframeDescriptors", false);
}

void VisualOdometryInterface::SetRobotInfo(RobotInfo::Ptr roboinfo)
{
    mSetHeight = roboinfo->AddHeightToOdometry();
    mRobotHeight = roboinfo->GetConstantHeight();
}


} // end namespace common
} // end namespace KSRobot

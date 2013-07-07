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
    mMaxKeyframesDist(DEFAULT_KEYFRAMES_DIST),
    mMaxKeyframesAngle(DEFAULT_KEYFRAMES_ANGLE),
    mRobotHeight(DEFAULT_ROBOT_HEIGHT),
    mProjectOnGround(false),
    mIsCycleKeyframe(true), // first cycle is always keyframe
    mLastKinectCycle(-1), mOdomTimer(new Timer("Odometry time"))
{
    mMotion = new MotionInfo;
    
    if( mProjectOnGround )
        mMotion->GlobalPose.translation()[1] = mRobotHeight;

    RegisterTimer(mOdomTimer);
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
        mKeyframeReceivers(GetLastKeyframe());
}

void VisualOdometryInterface::FinishCycle()
{
    Interface::FinishCycle();
    
    if( mProjectOnGround )
        ProjectToGround(); // only necessary for mMotionEstimate
    
    mMotion->GlobalPose = mMotion->GlobalPose * mMotion->MotionEstimate;
    
    CheckForKeyframe();
    if( IsThisCycleKeyframe() )
    {
        Keyframe kf;
        kf.GlobalPose = mMotion->GlobalPose;
        kf.RelativeMotion = mMotion->LastKeypointPose.inverse() * mMotion->GlobalPose;
        AddKeyframeFeatures(kf);
        mKeyframes.push_back(kf);
        mMotion->CurrRelativeMotion.setIdentity();
        mMotion->LastKeypointPose = mMotion->GlobalPose;
    }
    else
    {
        mMotion->CurrRelativeMotion = mMotion->LastKeypointPose.inverse() * mMotion->GlobalPose;
    }
    NotifyKeyframeReceivers();
}

static double ExtractRotationAngle(const Eigen::Matrix3f& r)
{
    float r11, r13, r31, r33;
    
    r11 = r(0, 0);
    r13 = r(0, 2);
    r31 = r(2, 0);
    r33 = r(2, 2);
    
    return atan2f(r13, r33);
}

void VisualOdometryInterface::ProjectToGround()
{
    mMotion->CurrRelativeMotion.translation()[1] = 0;
    float tetha = ExtractRotationAngle(mMotion->CurrRelativeMotion.rotation());
    mMotion->CurrRelativeMotion.linear() = Eigen::AngleAxisf(tetha, Eigen::Vector3f::UnitY()).toRotationMatrix(); 
}

bool VisualOdometryInterface::CheckForKeyframe()
{
    mIsCycleKeyframe = false;
    if( GetCycle() == 0 )
    {
        mIsCycleKeyframe = true;
        return mIsCycleKeyframe;
    }
    
    if( mMotion->CurrRelativeMotion.translation().squaredNorm() >= mMaxKeyframesDist * mMaxKeyframesDist )
    {
        mIsCycleKeyframe = true;
        return mIsCycleKeyframe;
    }
    
    //TODO: Check for angle
    if( mProjectOnGround )
    {
        float r13, r33;
        const Eigen::Matrix3f& rot = mMotion->CurrRelativeMotion.rotation();
        //TODO: REPLACE THIS WITH ExtractRotationAngle after debugs
        
//         r11 = rot(0, 0);
        r13 = rot(0, 2);
//         r31 = rot(2, 0);
        r33 = rot(2, 2);
        
//         double dc = abs(r11) - abs(r33);
//         double ds = abs(r13) - abs(r31);
//         
//         std::cout << "(VisualOdometryInterface::CheckForKeyframe) dc = " << dc << " ds = " << ds << std::endl << std::flush;
        
        float tetha = atan2f(r13, r33);
        
        if( isnan(tetha) )
            std::cout << "tetha is nan r13=" << r13 << " r33=" << r33 << std::endl << std::flush;
        
        if( abs(tetha) >= mMaxKeyframesAngle * M_PI / 180 )
            mIsCycleKeyframe = true;
    }
    
    //std::cout << "Motion Estimate:\n" << mMotionEstimate.rotation() << std::endl << std::flush;
    
    Eigen::Matrix<float, 3, 1> euler = mMotion->MotionEstimate.rotation().eulerAngles(2, 1, 0);
    euler = euler * 180 / M_PI;
    //std::cout << "Motion Estimate:\n Yaw=" << euler(0, 0) << " Pitch=" << euler(1, 0) << " Roll= " << euler(2, 0) << std::endl;
    euler = mMotion->CurrRelativeMotion.rotation().eulerAngles(2, 1, 0);
    euler = euler * 180 / M_PI;
    //std::cout << "Relative motion:\n Yaw=" << euler(0, 0) << " Pitch=" << euler(1, 0) << " Roll= " << euler(2, 0) << std::endl;
    //std::cout << "RELATIVE MATRIX:\n" << mCurrRelativeMotion.matrix() << std::endl;
    //std::cout << "RELATIVE MATRIX:\n" << mMotion->CurrRelativeMotion.matrix() << std::endl;
    //std::cout << "=====================================================================\n\n";
    
    return mIsCycleKeyframe;
}

void VisualOdometryInterface::ReadSettings(ProgramOptions::Ptr po)
{
    Interface::ReadSettings(po);
    
    mMaxKeyframesDist = po->GetDouble("MaxKeyframesDistance", DEFAULT_KEYFRAMES_DIST);
    mMaxKeyframesAngle = po->GetDouble("MaxKeyframesAngle", DEFAULT_KEYFRAMES_ANGLE);
    mProjectOnGround = po->GetBool("ProjectOnGround", true);
    mRobotHeight = po->GetDouble("RobotSensorHeight", DEFAULT_ROBOT_HEIGHT);
    
    
    if( mProjectOnGround )
    {
        //mGlobalPose.translation()[1] = mRobotHeight; // for kinect, height is along y axis.
    }
}

void VisualOdometryInterface::WriteSettings(ProgramOptions::Ptr po)
{
    Interface::WriteSettings(po);
    
    po->PutDouble("MaxKeyframesDistance", mMaxKeyframesDist);
    po->PutDouble("MaxKeyframesAngle", mMaxKeyframesAngle);
    po->PutBool("ProjectOnGround", mProjectOnGround);
    po->PutDouble("RobotSensorHeight", mRobotHeight);
}



} // end namespace common
} // end namespace KSRobot

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

VisualOdometryInterface::VisualOdometryInterface(const std::string& name): Interface(name),
    mMotionEstimate(Eigen::Isometry3d::Identity()), mCurrRelativeMotion(Eigen::Isometry3d::Identity()),
    mLastKeypointPose(Eigen::Isometry3d::Identity()),
    mMaxKeyframesDist(DEFAULT_KEYFRAMES_DIST),
    mMaxKeyframesAngle(DEFAULT_KEYFRAMES_ANGLE),
    mRobotHeight(DEFAULT_ROBOT_HEIGHT),
    mProjectOnGround(true),
    mIsCycleKeyframe(true), // first cycle is always keyframe
    mLastKinectCycle(-1), mOdomTimer(new Timer("Odometry time"))
{
    mGlobalPose.translation()[1] = mRobotHeight;
    RegisterTimer(mOdomTimer);
}

VisualOdometryInterface::~VisualOdometryInterface()
{
}

void VisualOdometryInterface::RegisterToKinect(KinectInterface::Ptr ki)
{
    mKinect = ki;
}

bool VisualOdometryInterface::RunSingleCycle()
{
    Interface::ScopedLock lock(this);
    mKinect->LockData();
    
    if( mKinect->GetCycle() <= mLastKinectCycle )
    {
        mKinect->UnlockData();
        return false;
    }
    
    mCurrFloatDepth = mKinect->GetFloatDepthImage();
    mCurrRawDepth = mKinect->GetRawDepthImage();
    mCurrRgb = mKinect->GetRgbImage();
    mCurrPointCloud = mKinect->GetPointCloud();
    
    mLastKinectCycle = mKinect->GetCycle();
    mKinect->UnlockData();
    return true;
}

void VisualOdometryInterface::NotifyKeyframeReceivers()
{
    if( IsThisCycleKeyframe() )
    {
        mKeypointReceivers();
    }
}

void VisualOdometryInterface::FinishCycle()
{
    Interface::FinishCycle();
    
    if( mProjectOnGround )
        ProjectToGround(); // only necessary for mMotionEstimate
    
    mGlobalPose = mGlobalPose * mMotionEstimate;
    mCurrRelativeMotion = mCurrRelativeMotion * mMotionEstimate;
    
    CheckForKeyframe();
    if( IsThisCycleKeyframe() )
    {
        Keypoint kp;
        kp.GlobalPose = mGlobalPose;
        kp.RelativeMotion = mLastKeypointPose.inverse() * mGlobalPose;
        mKeypoints.push_back(kp);
        mCurrRelativeMotion = Eigen::Isometry3d::Identity();
        mLastKeypointPose = mGlobalPose;
    }
    else
    {
        mCurrRelativeMotion = mLastKeypointPose.inverse() * mGlobalPose;
    }
    NotifyKeyframeReceivers();
}

static double ExtractRotationAngle(const Eigen::Matrix3d& r)
{
    double r11, r13, r31, r33;
    
    r11 = r(0, 0);
    r13 = r(0, 2);
    r31 = r(2, 0);
    r33 = r(2, 2);
    
    return atan2(r13, r33);
}

void VisualOdometryInterface::ProjectToGround()
{
//     Eigen::Vector3d position = mCurrRelativeMotion.translation();
//     Eigen::Matrix3d rot;
//     position[1] = 0; // set y change to zero.
//     double tetha = ExtractRotationAngle(mCurrRelativeMotion.rotation());
// 
//     rot = Eigen::AngleAxisd(tetha, Eigen::Vector3d::UnitY());
//     
//     Eigen::Isometry3d newTrans;
//     newTrans.translate(position);
//     newTrans.rotate(rot);
//     
//     mCurrRelativeMotion = newTrans;
    mCurrRelativeMotion.translation()[1] = 0; // Set change in this direction to zero
    double tetha = ExtractRotationAngle(mCurrRelativeMotion.rotation());
    mCurrRelativeMotion.linear() = Eigen::AngleAxisd(tetha, Eigen::Vector3d::UnitY()).toRotationMatrix();
    
    
}

void VisualOdometryInterface::CheckForKeyframe()
{
    if( GetCycle() == 0 )
    {
        mIsCycleKeyframe = true;
        return;
    }
    
    if( mCurrRelativeMotion.translation().squaredNorm() >= mMaxKeyframesDist * mMaxKeyframesDist )
    {
        mIsCycleKeyframe = true;
        return;
    }
    
    //TODO: Check for angle
    if( mProjectOnGround )
    {
        double r11, r13, r31, r33;
        const Eigen::Matrix3d& rot = mCurrRelativeMotion.rotation();
        
        //TODO: REPLACE THIS WITH ExtractRotationAngle after debugs
        
        r11 = rot(0, 0);
        r13 = rot(0, 2);
        r31 = rot(2, 0);
        r33 = rot(2, 2);
        
        double dc = abs(r11) - abs(r33);
        double ds = abs(r13) - abs(r31);
        
        std::cout << "(VisualOdometryInterface::CheckForKeyframe) dc = " << dc << " ds = " << ds << std::endl << std::flush;
        
        double tetha = atan2(r13, r33);
        
        if( abs(tetha) >= mMaxKeyframesAngle * M_PI / 180 )
            mIsCycleKeyframe = true;
        
        std::cout << "Tetha " << tetha * 180 / M_PI << "  Is keyframe " << mIsCycleKeyframe << std::endl << std::flush;
    }
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
        mGlobalPose.translation()[1] = mRobotHeight; // for kinect, height is along y axis.
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

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

namespace KSRobot
{
namespace common
{

VisualOdometryInterface::VisualOdometryInterface(const std::string& name): Interface(name),
    mMotionEstimate(Eigen::Isometry3d::Identity()), mCurrRelativeMotion(Eigen::Isometry3d::Identity()),
    mLastKinectCycle(-1), mOdomTimer(new Timer("Odometry time"))
{
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


} // end namespace common
} // end namespace KSRobot

/*
 * <one line to give the library's name and an idea of what it does.>
 * Copyright (C) 2013  Kourosh <kourosh.sartipi@gmail.com>
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

#include <interfaces/ICPInterface.h>

namespace KSRobot
{
namespace interfaces
{

ICPInterface::ICPInterface(const std::string& name): common::VisualOdometryInterface(name)
{
    SetWithNormals(false);
}

ICPInterface::~ICPInterface()
{
}

bool ICPInterface::Converged()
{
    return false;
}

float ICPInterface::GetConvergenceError()
{
    return 0;
}

bool ICPInterface::RunSingleCycle()
{
    if( common::VisualOdometryInterface::RunSingleCycle() == false )
        return false;
    
    LockData();
    
        if( mLastPointCloud.get() == 0 )
        {
            mLastPointCloud = mCurrPointCloud;
            return false; // can produce data in the next frame
        }
        
        mICP->setInputCloud(mLastPointCloud);
        mICP->setInputTarget(mCurrPointCloud);
        
        mOdomTimer->Start();
            mICP->align(mAlignedCloud);
            
            mMotionEstimate = mICP->getFinalTransformation().cast<double>();
            mGlobalPose = mGlobalPose * mMotionEstimate;            
        mOdomTimer->Stop();
        
        IncrementCycle();
    
    UnlockData();
    return true;
}

bool ICPInterface::IsThisCycleKeyframe()
{
    return true; // for now. //TODO: Change this later. // and possibly move it to visual odometry class
}

void ICPInterface::SetWithNormals(bool b)
{
    typedef pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBA, pcl::PointXYZRGBA>  ICPNormalsType;
    mWithNormals = b;
    if( mWithNormals )
    {
        //mICP.reset(new ICPNormalsType);
    }
    else
    {
        mICP.reset(new ICPAlgo);;
    }
}



};
};


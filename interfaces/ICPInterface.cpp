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
#include <pcl/registration/icp.h>

namespace KSRobot
{
namespace interfaces
{

class ICPInterface::Impl
{
public:
    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA>    ICP;
};
    
ICPInterface::ICPInterface(): common::VisualOdometryInterface(), mImpl(new Impl)
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

    Interface::ScopedLock lock(this);
    
    if( mLastPointCloud.get() == 0 )
    {
        mLastPointCloud = mCurrPointCloud;
        return false; // can produce data in the next frame
    }
    
    mImpl->ICP.setInputSource(mLastPointCloud);
    mImpl->ICP.setInputTarget(mCurrPointCloud);
    
    mOdomTimer->Start();
        mImpl->ICP.align(mAlignedCloud);
        //mMotionEstimate = mImpl->ICP.getFinalTransformation();
        mMotion->MotionEstimate = mImpl->ICP.getFinalTransformation();
    mOdomTimer->Stop();
    
    FinishCycle();
    
    return true;
}

void ICPInterface::SetWithNormals(bool b)
{
//     typedef pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBA, pcl::PointXYZRGBA>  ICPNormalsType;
    mWithNormals = b;
//     if( mWithNormals )
//     {
//         //mICP.reset(new ICPNormalsType);
//     }
//     else
//     {
//         mICP.reset(new ICPAlgo);;
//     }
}



};
};


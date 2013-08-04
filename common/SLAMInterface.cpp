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

#include <common/SLAMInterface.h>
#include <gtsam/global_includes.h>

namespace KSRobot
{
namespace common
{

SLAMInterface::SLAMInterface() : Interface(), mLastKeyframe(-1)
{
//     mLoops.set_capacity(1000);
//     mKeyframes.set_capacity(1000);
    
}

SLAMInterface::~SLAMInterface()
{
}

void SLAMInterface::RegisterToVO(VisualOdometryInterface::Ptr vo)
{
    mVO = vo;
    mConnections.push_back(mVO->RegisterKeyframeReceiver(boost::bind(&SLAMInterface::OnKeyframeDetected, this, _1)));
}

void SLAMInterface::ReadFromFile(const std::string& filename)
{
    //TODO: Implement
}

void SLAMInterface::OnLoopDetected(const LoopDetector::LoopClosure& lc)
{
    //mLoops.push(lc);
    SLAMDataArrival da;
    da.IsLoopClosure = true;
    da.Loop = lc;
    mUnprocessedData.push(da);
}

void SLAMInterface::RegisterToLoopDetector(LoopDetector::Ptr ld)
{
    ld->RegisterLoopReceiver(boost::bind(&SLAMInterface::OnLoopDetected, this, _1));
}

void SLAMInterface::OnKeyframeDetected(const VisualKeyframe::Ptr kf)
{
    //mKeyframes.push(kf);
    SLAMDataArrival da;
    da.IsLoopClosure = false;
    da.Keyframe = kf;
    mUnprocessedData.push(da);
}

bool SLAMInterface::RunSingleCycle()
{
    common::Interface::ScopedLock lock(this);
    
    int count = 0;
//     common::VisualKeyframe::Ptr kf;
//     while( mKeyframes.try_pop(kf) )
//     {
//         count++;
//         AddKeyframe(kf);
//         FinishCycle();
//     }
//     
//     common::LoopDetector::LoopClosure lc;
//     while( mLoops.try_pop(lc) )
//     {
//         count++;
//         AddLoopClosure(lc);
//         FinishCycle();
//     }
    
    SLAMDataArrival da;
    while( mUnprocessedData.try_pop(da) )
    {
        count++;
        if( da.IsLoopClosure )
            AddLoopClosure(da.Loop);
        else
            AddKeyframe(da.Keyframe);
        
        FinishCycle();
    }
    
    return count > 0;
}

} // end namespace common
} // end namespace KSRobot

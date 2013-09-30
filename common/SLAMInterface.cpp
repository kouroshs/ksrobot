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
    (void)filename;
}

void SLAMInterface::OnLoopDetected(const LoopDetectorInterface::LoopClosure& lc)
{
    SLAMDataArrival da;
    da.IsLoopClosure = true;
    da.Loop = lc;
    mUnprocessedData.push(da);
}

void SLAMInterface::RegisterToLoopDetector(LoopDetectorInterface::Ptr ld)
{
    ld->RegisterLoopReceiver(boost::bind(&SLAMInterface::OnLoopDetected, this, _1));
}

void SLAMInterface::OnKeyframeDetected(const VisualKeyframe::Ptr kf)
{
    SLAMDataArrival da;
    da.IsLoopClosure = false;
    da.Keyframe = kf;
    mUnprocessedData.push(da);
//     Debug("SLAMInterface::OnKeyframeDetected\n");
}

bool SLAMInterface::RunSingleCycle()
{
    int count = 0;
    SLAMDataArrival da;
    while( mUnprocessedData.try_pop(da) )
    {
        common::Interface::ScopedLock lock(this);
        count++;
        if( da.IsLoopClosure )
            AddLoopClosure(da.Loop);
        else
            AddKeyframe(da.Keyframe);
        
        FinishCycle();
    }
    
    return count > 0;
}

void SLAMInterface::FinishCycle()
{
    Update();
    common::Interface::FinishCycle();
}

} // end namespace common
} // end namespace KSRobot

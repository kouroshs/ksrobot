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

#include <interfaces/iSAM2Interface.h>
#include <gtsam/nonlinear/ISAM2.h>


namespace KSRobot
{
namespace interfaces
{

iSAM2Interface::iSAM2Interface() : common::SLAMInterface(), mLastestCycle(0)
{
    SetInterfaceName("iSAM2Interface");
    Initialize();
}

iSAM2Interface::~iSAM2Interface()
{
}

void iSAM2Interface::Initialize()
{
    //TODO: Change these by model
    mISAM2 = new gtsam::ISAM2();
    mPriorNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector_(1,0,0,1));
    mOdometryNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector_(1,0,0,1));
}


void iSAM2Interface::ReadSettings(common::ProgramOptions::Ptr po)
{
    //TODO:
    
    Initialize();
}

void iSAM2Interface::AddKeyframe(const common::VisualKeyframe::Ptr kf)
{
    //TODO: IMPLEMENT
    bool ref_frame_found = (mSeenCycles.find(kf->ReferenceCycle) != mSeenCycles.end());
    
    if( !ref_frame_found && mLastestCycle == 0 )
    {
        // this is the first cucle
        
    }
    else if( !ref_frame_found )
    {
        // This means this is not the first cycle, but reference frame was not found.
        // What this means is that the reference frame was changed by the VO algorithm
        // internally but the change was not registered as keyframe e.g. when FOVIS fails
        // to calculate motion from the reference frame but can calculate it using previous frame.
        
        //NOTE: in this case, since there are no nodes for the ref_frame, we should ignore unfortunately.
        Debug("Reference frame %d was not found.\n", kf->ReferenceCycle);
        return;
    }
    else
    {
        assert(ref_frame_found == true);
        // This means it is not the first cycle, and reference frame was found. In this case
        // we should add a node to the graph for this frame (the reference frame is already in the graph)
        // and we should add feature matches between this frame and reference frame.
    }
}

void iSAM2Interface::AddLoopClosure(const common::LoopDetector::LoopClosure& lc)
{
    //TODO:
    // NOTE: For loop closure, both frames should have been visited and exist inside the graph.
    if( mSeenCycles.find(lc.Cycle1) == mSeenCycles.end() || mSeenCycles.find(lc.Cycle2) == mSeenCycles.end() )
    {
        // THIS IS BAD
        if( lc.Cycle1 > mLastestCycle || lc.Cycle2 > mLastestCycle )
        {
            // This means we have cycles not registered to the graph.
            //TODO: We should add it to the queue again, but it could cause infinite loop.
            Error("Unknown loop closure cycles (%lu, %lu) were passed. Latest Cycle = %lu.\n", lc.Cycle1, lc.Cycle2, mLastestCycle);
        }
        return; // in any case, do no furthur processing.
    }
    //TODO: IMPLEMENT
}

void iSAM2Interface::Update()
{
    //TODO: Implement;
}


};
};

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

#include <common/LoopDetectorInterface.h>

namespace KSRobot
{
namespace common
{

LoopDetectorInterface::LoopDetectorInterface() : mPointCloudDownsampleRate(4)
{
    mGICP.SetDownsampleRate(1);
}

LoopDetectorInterface::~LoopDetectorInterface()
{
    if( mConnection.connected() )
        mConnection.disconnect();
}

void LoopDetectorInterface::ReadSettings(ProgramOptions::Ptr po)
{
    Interface::ReadSettings(po);
}

boost::signals2::connection LoopDetectorInterface::RegisterLoopReceiver(boost::function<void(const LoopClosure& lc)> fn)
{
    return mOnLoopDetected.connect(fn);
}

void LoopDetectorInterface::RegisterToVO(VisualOdometryInterface::Ptr vo)
{
    mVO = vo;
    
    if( mConnection.connected() )
        mConnection.disconnect();
    
    mConnection = mVO->RegisterKeyframeReceiver(boost::bind(&LoopDetectorInterface::OnNewKeyframe, this));
}

void LoopDetectorInterface::OnNewKeyframe()
{
    KeyframeData kd;
    kd.Keyframe = mVO->GetLatestKeyframe();
    kd.Image = mVO->GetCurrentGrayImage();
    kd.RawDepth = mVO->GetCurrentRawDepthImage();
    kd.PointCloud = mVO->GetCurrentPointCloud();
    
    mQueue.push(kd);
}

bool LoopDetectorInterface::RunSingleCycle()
{
    KeyframeData kd;
    int count = 0;
    std::vector<size_t> candidates;
    Eigen::Isometry3f trans;
    
    while( mQueue.try_pop(kd) )
    {
        PreprocessFrame(kd);
        ProcessFrame(kd);
        if( GetCycle() > 1 )
        {
            // check for loop closure
            candidates.clear();
            CollectCandidates(kd.Keyframe->CurrentCycle, candidates);
            
            // now for each candidate, check if we can find tranforma between the two. If we could, stop.
            bool foundLoop = false;
            size_t i;
            for(i = 0; i < candidates.size(); i++)
                if( (foundLoop = CheckTransform(candidates[i], kd.Keyframe->CurrentCycle, trans)) )
                    break;
            
            if( foundLoop )
            {
                LoopClosure lc;
                lc.Cycle1 = candidates[i];
                lc.Cycle2 = kd.Keyframe->CurrentCycle;
                lc.Transform = trans;
                
                mOnLoopDetected(lc);
            }
        }
        
        FinishCycle();
    }
    
    return count > 0;
}

void LoopDetectorInterface::PreprocessFrame(const LoopDetectorInterface::KeyframeData& kd)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    PCLUtils::DownsampleOrganized(kd.PointCloud, pc, mPointCloudDownsampleRate);
    
    if( mPointCloudMap.find(kd.Keyframe->CurrentCycle) != mPointCloudMap.end() )
    {
        //NOTE: THIS IS A SERIOUS ERROR
        throw std::runtime_error("(LoopDetectorInterface::PreprocessFrame) the current cycle has already been seen. This might be caused by a bug at VisualOdometryInterface or its derived classes.");
    }
    
    mPointCloudMap[kd.Keyframe->CurrentCycle] = pc;
}

bool LoopDetectorInterface::CheckTransform(size_t prev_cycle, size_t curr_cycle, Eigen::Isometry3f& trans)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr prev, curr;
    PointCloudMap::iterator iter_prev = mPointCloudMap.find(prev_cycle), iter_curr = mPointCloudMap.find(curr_cycle);
    
    if( iter_prev == mPointCloudMap.end() )
        throw std::runtime_error("(LoopDetectorInterface::CheckTransform) BUG: Previous cycle has not been seen.");
    if( iter_curr == mPointCloudMap.end() )
        throw std::runtime_error("(LoopDetectorInterface::CheckTransform) BUG: Current cycle has not been seen.");
    
    prev = iter_prev->second;
    curr = iter_curr->second;
    
    mGICP.SetInputSource(prev);
    mGICP.SetInputTraget(curr);
    
    mGICP.ComputeTransform();
    trans = mGICP.GetFinalComputedTransform();    
    return mGICP.Converged();
}

};
};

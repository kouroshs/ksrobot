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

#include <interfaces/FovisInterface.h>
#include <math.h>
#include <limits>
#include <fovis/visual_odometry.hpp>
#include <fovis/depth_image.hpp>

// #include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>

#define SAFE_DELETE(x) if(x) { delete (x); (x) = NULL; }

namespace KSRobot
{
namespace interfaces
{

FovisInterface::FovisInterface() : 
            common::VisualOdometryInterface(), 
            mFovis(NULL), mDepthImage(NULL), mRectification(NULL), mNextCycleKeyframe(false),
            mUseIcpOnFail(true),
            mPrevFovisReferenceFrame(0)
{
    SetInterfaceName("FovisInterface");
}

FovisInterface::~FovisInterface()
{
    SAFE_DELETE(mFovis);
    SAFE_DELETE(mRectification);
    SAFE_DELETE(mDepthImage);
}

void FovisInterface::InitInternal(const common::KinectInterface::CameraParameters& kinectParams, const fovis::VisualOdometryOptions& options)
{
    mOptions = options;
    fovis::CameraIntrinsicsParameters camParams;
    memset(&camParams, 0, sizeof(camParams));
    
    camParams.width = kinectParams.Width;
    camParams.height = kinectParams.Height;
    camParams.fx = kinectParams.FocalX;
    camParams.fy = kinectParams.FocalY;
    camParams.cx = kinectParams.CenterX;
    camParams.cy = kinectParams.CenterY;
    camParams.k1 = kinectParams.K1;
    camParams.k2 = kinectParams.K2;
    camParams.k3 = kinectParams.K3;
    camParams.p1 = kinectParams.P1;
    camParams.p2 = kinectParams.P2;

    SAFE_DELETE(mFovis);
    SAFE_DELETE(mRectification);
    SAFE_DELETE(mRectification);
    
    mRectification  = new fovis::Rectification(camParams);
    mFovis          = new fovis::VisualOdometry(mRectification, mOptions);
    mDepthImage     = new fovis::DepthImage(camParams, camParams.width, camParams.height);
}

void FovisInterface::ReadSettings(common::ProgramOptions::Ptr po)
{
    common::VisualOdometryInterface::ReadSettings(po);
    mUseIcpOnFail = po->GetBool("UseIcpOnFail", true);
}

void FovisInterface::RegisterToKinect(common::KinectInterface::Ptr ki)
{
    assert(ki.get());
    VisualOdometryInterface::RegisterToKinect(ki);
    InitInternal(mKinect->GetCameraParams(), fovis::VisualOdometry::getDefaultOptions());
}

bool FovisInterface::RunSingleCycle()
{
    if( VisualOdometryInterface::RunSingleCycle() == false )
        return false;
    // now we have the kinect data.
    Interface::ScopedLock fovisLock(this);
    
    mDepthImage->setDepthImage(mCurrFloatDepth->GetArray().data());
    
    mOdomTimer->Start();
        mFovis->processFrame(mCurrGray->GetArray().data(), mDepthImage);
        //NOTE:
        mConverged = false;
        
        if( mFovis->getMotionEstimateStatus() == fovis::SUCCESS )
        {
            mMotion->MotionEstimate = mFovis->getMotionEstimate().cast<float>();
            mConverged = true;
        }
        else if( GetCycle() > 2 )
        {
            if( mUseIcpOnFail )
            {
                mGICP.SetInputSource(mPrevPointCloud);
                mGICP.SetInputTraget(mCurrPointCloud);
                mGICP.ComputeTransform();
                
                if( mGICP.Converged() )
                {
                    mMotion->MotionEstimate = mGICP.GetFinalComputedTransform();
                    mConverged = true;
                }
            }
        }
    mOdomTimer->Stop();

    FinishCycle();
    
    return true;
}

float FovisInterface::GetConvergenceError()
{
    return 0;
}

void FovisInterface::FinishCycle()
{
    if( mNextCycleKeyframe )
    {
        //NOTE: The code moved from CheckForKeyframe
        //TODO: Since fovis sets the reference frame to PREVIOUS frame, check if we should
        //      set it to GetCycle() - 1 or not.
        //mNextCycleKeyframe = false;
        //mIsCycleKeyframe = true;
        mPrevFovisReferenceFrame = GetCycle(); // now future frames will reference to this frame
    }
    
    common::VisualOdometryInterface::FinishCycle();
}

bool FovisInterface::CheckForKeyframe()
{
    if( mNextCycleKeyframe )
    {
        //NOTE: I should set this variable to false, but since this should also change the mPrevFovisReferenceFrame
        //      to the current cycle, but PublishKeyframeFeatures needs that var to represent the previous reference frame,
        //      I've moved this portion of the code to PublishKeyframeFeatures.
//         Debug("(FovisInterface::CheckForKeyframe) next cycle keyframe.\n");
        mNextCycleKeyframe = false;
        mIsCycleKeyframe = true;
        return true;
    }
    
    if( VisualOdometryInterface::CheckForKeyframe() )
        return true;
    if( mFovis->getChangeReferenceFrames() )
    {
        mNextCycleKeyframe = true; //NOTE: This cycle is not keyframe, but the next cycle will be.
    }
    return false;
}

void FovisInterface::PublishKeyframeFeatures(common::VisualKeyframe::Ptr kf)
{
//     Debug("(FovisInterface::PublishKeyframeFeatures)\n");
    // This function extracts feature data from fovis.
    const fovis::OdometryFrame* curr_frame = mFovis->getTargetFrame();
    //NOTE: Extract features only for the first pyramid.
    const fovis::PyramidLevel* pyr = curr_frame->getLevel(0);
    kf->DataPool.FeatureLength = pyr->getDescriptorLength();
    kf->DataPool.FeatureStride = pyr->getDescriptorStride();
    
    if( KeyframeDescriptorPublishingEnabled() )
        kf->DataPool.resize(kf->DataPool.FeatureStride * pyr->getNumKeypoints());

    size_t num_saved_keypoints = 0;
    for(int i = 0; i < pyr->getNumKeypoints(); i++)
    {
        const fovis::KeypointData* data = pyr->getKeypointData(i);
        if( !data->has_depth )
            continue; // Only feature with depth are allowed.
        
        common::VisualFeature f;
        f.Score = data->kp.score;
        f.U = data->kp.u;
        f.V = data->kp.v;
        f.RelativePosition = data->xyz.cast<float>();
        
        kf->Features.push_back(f);
        //Now copy feature data.
        if( KeyframeDescriptorPublishingEnabled() )
            memcpy(kf->DataPool.data() + num_saved_keypoints * pyr->getDescriptorStride(), pyr->getDescriptor(i), pyr->getDescriptorLength());
        num_saved_keypoints++;
    }
//     Debug("\tKeypointPublishing: %s, Num saved keypoints:%d\n", KeyframeDescriptorPublishingEnabled() ? "true" : "false", num_saved_keypoints);
    
    if( KeyframeDescriptorPublishingEnabled() )
        kf->DataPool.resize(num_saved_keypoints * pyr->getDescriptorStride());
    
    // Now it's time to publish feature matches.
    
    const fovis::MotionEstimator* est = mFovis->getMotionEstimator();
    const fovis::FeatureMatch* matches_base = est->getMatches();
    common::VisualKeyframe::MatchedPair pair;
    
    kf->MatchedPairs.reserve(est->getNumInliers()); // only inliers will be considered.
    kf->CurrentCycle = GetCycle();
    
    if( mFovis->estimateUsingReferenceFrame() )
        kf->ReferenceCycle = mPrevFovisReferenceFrame;
    else
        kf->ReferenceCycle = GetCycle() - 1;
    
    for(int i = 0; i < est->getNumMatches(); i++)
    {
        const fovis::FeatureMatch* currMatch = matches_base + i;
        if( !currMatch->inlier )
            continue;
        
        pair.CurrentRelPosition = currMatch->target_keypoint->xyz.cast<float>();
        pair.ReferenceRelPosition = currMatch->ref_keypoint->xyz.cast<float>();
        pair.ReferenceIndex = currMatch->ref_keypoint->keypoint_index;
        kf->MatchedPairs.push_back(pair);
    }
}

} // end namespace utils
} // end namespace KSRobot

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
#include <fovis/visual_odometry.hpp>
#include <fovis/depth_image.hpp>

namespace KSRobot
{
namespace interfaces
{

class FovisInterface::FovisImpl
{
public:
    fovis::VisualOdometry*                      mFovis;
    fovis::DepthImage*                          mDepthImage;
    unsigned char*                              mGrayImage;
    
    fovis::VisualOdometryOptions                mOptions;
    fovis::Rectification*                       mRectification;
    
    FovisImpl(const common::KinectInterface::CameraParameters& kinectParams)
    {
        Init(kinectParams, fovis::VisualOdometry::getDefaultOptions());
    }
    
    ~FovisImpl()
    {
        delete mFovis;
        delete mDepthImage;
        delete mGrayImage;
        delete mRectification;
    }
    
    void Init(const common::KinectInterface::CameraParameters& kinectParams,
              const fovis::VisualOdometryOptions& options)
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
        
        mRectification = new fovis::Rectification(camParams);
        mFovis = new fovis::VisualOdometry(mRectification, mOptions);
        mDepthImage = new fovis::DepthImage(camParams, camParams.width, camParams.height);
        mGrayImage = new unsigned char[camParams.width * camParams.height];
    }
    
};
    
FovisInterface::FovisInterface(const std::string& name) : 
            common::VisualOdometryInterface(name), 
            mDataCopyTimer(new common::Timer("Data copying"))
{
    RegisterTimer(mDataCopyTimer);
}

FovisInterface::~FovisInterface()
{
}

void FovisInterface::RegisterToKinect(common::KinectInterface::Ptr ki)
{
    VisualOdometryInterface::RegisterToKinect(ki);
    mImpl.reset(new FovisImpl(ki->GetCameraParams()));
}

bool FovisInterface::RunSingleCycle()
{
    if( VisualOdometryInterface::RunSingleCycle() == false )
        return false;
    // now we have the kinect data.
    Interface::ScopedLock fovisLock(this);
    
    mDataCopyTimer->Start();
    
    mImpl->mDepthImage->setDepthImage(mKinect->GetFloatDepthImage()->GetArray().data());
    
    common::KinectRgbImage::ConstPtr rgb = mKinect->GetRgbImage();
    size_t size = rgb->GetHeight() * rgb->GetWidth();
    
    const common::KinectRgbImage::ArrayType& array = rgb->GetArray();
    
    for(size_t i = 0; i < size; i++)
    {
        size_t startIndex = i * 3;
        unsigned char r, g, b;
        r = array[startIndex];
        g = array[startIndex + 1];
        b = array[startIndex + 2];
        
        mImpl->mGrayImage[i] = (int)roundf(0.2125 * r + 0.7154 * g + 0.0721 * b);
    }
    mDataCopyTimer->Stop();

    mOdomTimer->Start();
        mImpl->mFovis->processFrame(mImpl->mGrayImage, mImpl->mDepthImage);
        mMotionEstimate = mImpl->mFovis->getMotionEstimate();
        //mGlobalPose = mImpl->mFovis->getPose();
    mOdomTimer->Stop();
    
    FinishCycle();
    
    return true;
}

bool FovisInterface::Converged()
{
    return mImpl->mFovis->getMotionEstimateStatus() == fovis::SUCCESS;
}

float FovisInterface::GetConvergenceError()
{
    return 0;
}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif//M_PI

void FovisInterface::CheckForKeyframe()
{
    VisualOdometryInterface::CheckForKeyframe();
    if( mIsCycleKeyframe )
        return; // no need to check anymore.
    
    if( mImpl->mFovis->getChangeReferenceFrames() )
        mIsCycleKeyframe = true; // TODO: Should I make this cycle keyframe or the next one?
}


} // end namespace utils
} // end namespace KSRobot

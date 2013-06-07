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

namespace KSRobot
{
namespace interfaces
{

FovisInterface::FovisInterface(common::ProgramOptions::Ptr po, const std::string& name) : 
            common::VisualOdometryInterface(po, name), mLastKinectCycle(-1)
{
}

FovisInterface::~FovisInterface()
{
}

void FovisInterface::RegisterToKinect(common::KinectInterface::Ptr ki)
{
    mKinect = ki;
    //TODO: COMPLETE THIS.
    fovis::VisualOdometryOptions opts;
    fovis::CameraIntrinsicsParameters camParams;
    int w, h;
    
    mFovis.reset(new fovis::VisualOdometry(NULL, opts));
    mDepthImage.reset(new fovis::DepthImage(camParams, w, h));
    mGrayImage.reset(new unsigned char[w * h]);
}

bool FovisInterface::RunSingleCycle()
{
    //Interface::ScopedLock kinectLock(mKinect.get());
    Interface::ScopedLock fovisLock(this);
    //TODO: Change fovis to make a local copy and do it fast!
    
    mKinect->LockData();
    if( mKinect->GetCycle() > mLastKinectCycle )
    {
        mDepthImage->setDepthImage(mKinect->GetFloatDepthImage()->GetArray().data());
        
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
            
            mGrayImage[i] = (int)roundf(0.2125 * r + 0.7154 * g + 0.0721 * b);
        }
        mKinect->UnlockData();
        
        mFovis->processFrame(mGrayImage.get(), mDepthImage.get());
        
        return true;
    }
    mKinect->UnlockData();
    // No new data
    return false;
}



} // end namespace utils
} // end namespace KSRobot

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

#include <interfaces/MTEngine.h>
#include <interfaces/KinectDatasetReader.h>
#include <interfaces/KinectDeviceReader.h>
#include <interfaces/FovisInterface.h>
#include <interfaces/ICPInterface.h>

namespace KSRobot
{
namespace interfaces
{

MTEngine::MTEngine() : common::EngineInterface()
{
}

MTEngine::~MTEngine()
{
}

void MTEngine::Initialize()
{
    common::EngineInterface::Initialize();
    
    if( mExecCtrl.Kinect.GetFromDevice )
    {
        mKinect.reset(new KinectDeviceReader());
        mKinect->Initialize(mExecCtrl.Kinect.SourceDevice);
    }
    else
    {
        mKinect.reset(new KinectDatasetReader());
        mKinect->Initialize(mExecCtrl.Kinect.SourceDir);
    }
    
    //NOTE: ICP visual odometry is only for testing, so there is no need to implement differently
    mVO.reset(new FovisInterface());
    //mVO.reset(new ICPInterface("ICP"));
    mVO->RegisterToKinect(mKinect);
    
    //Set HZ
    //TODO: Add these params to ProgramOptions?
    mKinect->SetHZ(33); // this is only for dataset reader
    mVO->SetHZ(5);
}

void MTEngine::Start()
{
    //DO NOT CALL EngineInterface::Start, we are not using a thread for engine execution!
    mKinect->Start();
    mVO->Start();
    mContinueExec = true;
}

void MTEngine::Stop()
{
    //DO NOT CALL EngineInterface::Stop
    if( mContinueExec == false )
        return;
    
    mContinueExec = false;
    mVO->Stop();
    mKinect->Stop();
    
}

bool MTEngine::RunSingleCycle()
{
    return false; // forcing a single cycle run on a multithreaded engine has no meaning
}


} // end namespace common
} // end namespace KSRobot

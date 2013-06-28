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

#include <iostream>

#include <gui/LogicBridge.h>
#include <gui/Utils.h>

#include <interfaces/MTEngine.h>
#include <interfaces/SerialEngine.h>

#include <boost/filesystem.hpp>

#include <exception>
#include <sstream>

using std::cout;
using std::flush;
using std::endl;

namespace KSRobot
{
namespace gui
{

LogicBridge::LogicBridge(QObject* parent): QObject(parent), mNumKinectReceiversConnected(0),
    mRGBDEnabled(true), mPointCloudEnabled(true)
{
}

LogicBridge::~LogicBridge()
{
}

QString LogicBridge::GetSavePath() const
{
    return mSavePath;
}

void LogicBridge::SaveKinectInputTo(const QString& path)
{
    mSavePath = path;
}

void LogicBridge::OnStart(const common::ExecCtrlData& data)
{
    try
    {
        if( mEngine.get() )
            mEngine->Stop();
        mEngine.reset(new interfaces::MTEngine());
        mEngine->SetExecutionParams(data);
        mEngine->Initialize();
        
        assert(mEngine->GetKinectInterface().get());
        assert(mEngine->GetVisualOdometryInterface().get());
        
        mEngine->GetKinectInterface()->RegisterOnCycleCompleteReceiver(boost::bind(&LogicBridge::OnKinectNewDataReceive, this));
        mEngine->GetKinectInterface()->RegisterOnFinishReceiver(boost::bind(&LogicBridge::OnKinectFinish, this));
        mEngine->GetVisualOdometryInterface()->RegisterOnCycleCompleteReceiver(boost::bind(&LogicBridge::OnFovisCycleComplete, this));
    }
    catch(std::exception& ex)
    {
        emit OnError(QString(ex.what()));
        return;
    }
    catch(...)
    {
        emit OnError(QString("Unknown Error"));
        return;
    }
    
    try
    {
        mEngine->Start();
    }
    catch(std::exception& ex)
    {
        emit OnError(QString(ex.what()));
    }
    catch(...)
    {
        emit OnError(QString("(LogicBridge::OnStart) Unknown Error"));
    }
}

void LogicBridge::OnStop()
{
    OnKinectFinish();
}

void LogicBridge::connectNotify(const char* sig)
{
    QObject::connectNotify(sig);
    if( sig != NULL && strcmp(sig, SIGNAL(OnRGBD(QImage,QImage))) == 0 )
        mNumKinectReceiversConnected++;
}

void LogicBridge::disconnectNotify(const char* sig)
{
    QObject::disconnectNotify(sig);
    if( sig != NULL && strcmp(sig, SIGNAL(OnRGBD(QImage,QImage))) == 0 )
        mNumKinectReceiversConnected--;
}

void LogicBridge::OnKinectFinish()
{
    try
    {
        if( mEngine.get() )
            mEngine->Stop();
    }
    catch(std::exception& ex)
    {
        emit OnError(QString(ex.what()));
    }
    catch(...)
    {
        emit OnError(QString("(LogicBridge::OnKinectFinish) Unknown Error"));
    }
    emit ExecutionFinished();
}

void LogicBridge::OnKinectNewDataReceive()
{
    mEngine->GetKinectInterface()->LockData();
    common::KinectRgbImage::ConstPtr rgb = mEngine->GetKinectInterface()->GetRgbImage();
    common::KinectRawDepthImage::ConstPtr depth = mEngine->GetKinectInterface()->GetRawDepthImage();
    common::KinectPointCloud::ConstPtr pc = mEngine->GetKinectInterface()->GetPointCloud();
    mEngine->GetKinectInterface()->UnlockData();
    
    if( mSavePath != "" )
    {
        try
        {
            std::stringstream ssPartialFileName;
            common::TimePoint t = common::Clock::now();
            long nanosecs = common::Nanoseconds(t.time_since_epoch());
            double save_time = (double)nanosecs * 10e-9;
            boost::filesystem::path prgb(mSavePath.toStdString()), pdepth(mSavePath.toStdString());
            prgb /= "rgb";
            pdepth /= "depth";
            
            ssPartialFileName << std::fixed << std::setprecision(6);
            ssPartialFileName << save_time;
            
            prgb /= ssPartialFileName.str() + ".png";
            pdepth /= ssPartialFileName.str() + ".png";
            
            common::KinectImageDiskIO::SaveToFileRgb(prgb.string(), rgb);
            common::KinectImageDiskIO::SaveToFileDepth(pdepth.string(), depth);
        }
        catch(std::exception& ex)
        {
            std::cout << "(LogicBridge::OnKinectNewDataReceive) Exception occured: " << ex.what() << std::endl << std::flush;
        }
    }
    
    if( mNumKinectReceiversConnected && !mSkipPC.ShouldSkip() && mRGBDEnabled )
    {
        QImage qrgb, qdepth;
        qrgb = Utils::ConvertToQImage(rgb);
        qdepth = Utils::ConvertToQImage(depth);
        emit OnRGBD(qrgb, qdepth);
    }
    
    if( !mSkipPC.ShouldSkip() && mPointCloudEnabled )
        emit OnPointCloud(pc);
}

void LogicBridge::OnFovisCycleComplete()
{
    Eigen::Isometry3d motion = mEngine->GetVisualOdometryInterface()->GetMotionEstimate();
    Eigen::Vector3d v = motion.translation();
    emit OnVisualOdometry(QVector3D(v[0], v[1], v[2]));
}


} // end namespace gui
} // end namespace KSRobot


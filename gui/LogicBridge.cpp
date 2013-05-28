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

#include "LogicBridge.h"

namespace KSRobot
{
namespace gui
{

LogicBridge::LogicBridge(QObject* parent): QObject(parent)
{
    qRegisterMetaType<utils::KinectPointCloud::Ptr>("KinectPointCloud::Ptr");
    qRegisterMetaType<utils::KinectRgbImage::Ptr>("KinectRgbImage::Ptr");
    qRegisterMetaType<utils::KinectFloatDepthImage::Ptr>("KinectFloatDepthImage::Ptr");
    qRegisterMetaType<utils::KinectRawDepthImage::Ptr>("KinectRawDepthImage::Ptr");
}

LogicBridge::~LogicBridge()
{
}

QString LogicBridge::GetSavePath() const
{
    return mSavePath;
}

void LogicBridge::SaveKinectInput(const QString& path)
{
    mSavePath = path;
}


void LogicBridge::OnStart(const ExecControlData& data)
{
}

void LogicBridge::OnStop()
{
    
}

void LogicBridge::KinectPointCloudReceiverDirect(utils::KinectPointCloud::ConstPtr& pc)
{
    //TODO: Emit the signal
}

void LogicBridge::KinectRGBDReceiverFloatDirect(utils::KinectRgbImage::Ptr rgb,
                                                utils::KinectFloatDepthImage depth)
{
    //TODO: Emit the signal
}

void LogicBridge::KinectRGBDReveicerRawDirect(utils::KinectRgbImage::Ptr rgb,
                                              utils::KinectRawDepthImage::Ptr depth)
{
    //TODO: Emit the signal
}


} // end namespace gui
} // end namespace KSRobot


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

#include <utils/kinect/KinectDatasetReader.h>
#include <utils/kinect/KinectDeviceReader.h>

namespace KSRobot
{
namespace gui
{

LogicBridge::LogicBridge(QObject* parent): QObject(parent), mQtImageCreatorReceivers(0)
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

void LogicBridge::connectNotify(const char* name)
{
    QObject::connectNotify(name);
    if( QString(name) == SIGNAL(OnRGBD(QImage, QImage)) )
        mQtImageCreatorReceivers++;
}

void LogicBridge::disconnectNotify(const char* name)
{
    QObject::disconnectNotify(name);
    if( QString(name) == SIGNAL(OnRGBD(QImage, QImage)) )
        mQtImageCreatorReceivers--;
}


void LogicBridge::OnStart(const utils::ExecCtrlData& data)
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
    // Normally I should not register this.
}

void LogicBridge::KinectRGBDReveicerRawDirect(utils::KinectRgbImage::Ptr rgb,
                                              utils::KinectRawDepthImage::Ptr depth)
{
    //TODO: Emit the signal
    //Two chekcs, one to save, and one to emit signals
    if( mSavePath != "" )
    {
        //TODO: Save kinect data
    }
    
    if( mQtImageCreatorReceivers )
    {
        QImage qrgb(rgb->GetWidth(), rgb->GetHeight(), QImage::Format_RGB32), 
                qdepth(depth->GetWidth(), depth->GetHeight(), QImage::Format_RGB32);
        
        // Now fill the images
        //NOTE: Assuming that the rgb and depth images have the same size
        assert(rgb->GetHeight() == depth->GetHeight() &&
                rgb->GetWidth() == depth->GetWidth());
        
        
        const utils::KinectRgbImage::ArrayType& rgbArray = rgb->GetArray();
        const utils::KinectRawDepthImage::ArrayType& depthArray = depth->GetArray();
        
        for(int y = 0; y < rgb->GetHeight(); y++)
        {
            uchar* rgbPtr = qrgb.scanLine(y);
            uchar* depthPtr = qdepth.scanLine(y);
            
            size_t idxRgb = rgb->ScanLineIndex(y);
            size_t idxDepth = depth->ScanLineIndex(y);
            for(int x = 0; x < rgb->GetWidth(); x++)
            {
                // First do the rgb
                rgbPtr[0] = rgbArray[idxRgb];
                rgbPtr[1] = rgbArray[idxRgb + 1];
                rgbPtr[2] = rgbArray[idxRgb + 2];
                rgbPtr[3] = 0xFF;
                
                //Now convert depth values
                depthPtr[0] = depthArray[idxDepth] / 10000 * 255;
                depthPtr[1] = depthArray[idxDepth] / 10000 * 255;
                depthPtr[2] = depthArray[idxDepth] / 10000 * 255;
                depthPtr[3] = 0xFF;
                
                idxRgb = utils::KinectRgbImage::NexIndexUnsafe(idxRgb);
                idxDepth = utils::KinectRawDepthImage::NexIndexUnsafe(idxDepth);
                rgbPtr += 4;
                depthPtr += 4;
            }
        }
        
        emit OnRGBD(qrgb, qdepth);
    }
}


} // end namespace gui
} // end namespace KSRobot


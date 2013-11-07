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

#ifndef LOGICBRIDGE_H
#define LOGICBRIDGE_H

#include <QObject>
#include <QVector3D>

#include <gui/ExecutionControl.h>
#include <common/SettingsBinder.h>
#include <common/EngineInterface.h>
#include <common/ExecCtrlData.h>


#include <QPushButton>

namespace KSRobot
{
namespace gui
{

class LogicBridge : public QObject
{
    Q_OBJECT
public:
    explicit LogicBridge(QObject* parent = 0);
    ~LogicBridge();

    
    QPushButton* buttons[5];

public slots:    
    void b0();
    void b1();
    void b2();
    void b3();
    void b4();
    
    
//     void                                SaveKinectInputTo(const QString& path);
//     QString                             GetSavePath() const;
//     
//     common::EngineInterface::Ptr        GetEngine() { return mEngine; }
//     
//     void                                SetRGBDSkip(int skip) { mSkipRGBD.Skip = skip; }
//     int                                 GetRGBDSkip() const { return mSkipRGBD.Skip; }
//     
//     void                                SetPointCloudSkip(int s) { mSkipPC.Skip = s; }
//     int                                 GetPointCloudSkip() const { return mSkipPC.Skip; }
//     
//     inline void                         EnableRGBD(bool enable);
//     inline void                         EnablePointCloud(bool enable);
//     
// signals:
//     void                                ExecutionFinished();
//     void                                OnRGBD(QImage rgb, QImage depth);
//     void                                OnPointCloud(common::KinectPointCloud::ConstPtr pc);
//     void                                OnVisualOdometry(QVector3D motionEstimate);
//     void                                OnError(const QString& err);
// public slots:
//     void                                OnStart(const common::ExecCtrlData& data);
//     void                                OnStop();
//     
// private slots:
//     
//     
// protected:
//     virtual void                        connectNotify(const char* sig);
//     virtual void                        disconnectNotify(const char* sig);
//     
//     void                                OnKinectNewDataReceive();
//     void                                OnKinectFinish();
//     void                                OnFovisCycleComplete();
// protected:
//     class SkipParams
//     {
//     public:
//         int                             Skip;
//         int                             CurrCount;
//         
//         bool                            ShouldSkip()
//         {
//             if( Skip == 0 ) return false;
//             if( ++CurrCount == Skip )
//             {
//                 CurrCount = 0;
//                 return false;
//             }
//             return true;
//         }
//         
//         SkipParams() : Skip(0), CurrCount(0) {;}
//     };
//     
//     SkipParams                          mSkipRGBD;
//     SkipParams                          mSkipPC;
//     
//     int                                 mNumKinectReceiversConnected;
//     bool                                mRGBDEnabled;
//     bool                                mPointCloudEnabled;
//     common::SettingsBinder              mBinder;
//     QString                             mSavePath;
//     common::EngineInterface::Ptr        mEngine;
// };
// 
// inline void LogicBridge::EnablePointCloud(bool enable)
// {
//     mPointCloudEnabled = enable;
// }
// 
// inline void LogicBridge::EnableRGBD(bool enable)
// {
//     mRGBDEnabled = enable;
// }
// 
// 
// } // end namespace gui
// } // end namespace KSRobot
// 
};
#endif // LOGICBRIDGE_H

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

#ifndef ROBXCONTROLMODULE_H
#define ROBXCONTROLMODULE_H

#include <QObject>
#include <boost/shared_ptr.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <vector>
#include <queue>

class QTimer;


namespace KSRobot
{
namespace gui
{

class RobXControl : public QObject
{
    Q_OBJECT
public:
    
    enum RobXMotorCommand
    {
        CMD_GET_SPEED_1 = 0, 
        CMD_GET_SPEED_2, 
        CMD_GET_ENCODER_1, 
        CMD_GET_ENCODER_2, 
        CMD_GET_ENCODERS, 
        CMD_GET_VOLTS, 
        CMD_GET_CURRENT_1, 
        CMD_GET_CURRENT_2, 
        CMD_GET_VERSION, 
        CMD_GET_ACCELERATION, 
        CMD_GET_MODE, 
        CMD_GET_VI, 
        CMD_GET_ERROR, 
        CMD_SET_SPEED_1, 
        CMD_SET_SPEED_2, 
        CMD_SET_ACCELERATION, 
        CMD_SET_MODE, 
        CMD_RESET_ENCODERES, 
        CMD_DISABLE_REGULATOR, 
        CMD_ENABLE_REGULATOR, 
        CMD_DISABLE_TIMEOUT, 
        CMD_ENABLE_TIMEOUT
    };
    
    explicit RobXControl(QObject* parent = 0);
    virtual ~RobXControl();
    
    
    void                                Open(const QString& device);
    void                                Close();
    bool                                IsOpen();
    
    void                                Write(RobXMotorCommand cmd, unsigned char val1 = 0, unsigned char val2 = 0, 
                                              unsigned char val3 = 0);
    void                                Read(unsigned char* buffer, size_t numBytes);
    void                                Read(std::vector<unsigned char>& buffer, size_t numBytes);
    void                                Read(size_t numBytes);
    
    int                                 GetVoltage();
    int                                 GetVersion();
    
    int                                 GetCurrent(int index);
    
    void                                EnableTimeout(bool enable);
    void                                EnableAutoregulator(bool enable);
    
    const std::vector<unsigned char>    GetReadBuffer() const { return mReadBuffer; }
    const std::vector<unsigned char>    GetWriteBuffer() const { return mWriteBuffer; }
    
    float                               GetEncoderValue(int index) const;
    
signals:
    void                                OnCommandDone();
    
public slots:
    void                                Stop();
    void                                Forward(int cm, float speed_index);
    void                                Backward(int cm, float speed_index);
    
    void                                TurnLeft(int degree, float speed_index);
    void                                TurnRight(int degree, float speed_index);
    
private slots:
    void                                OnMoveTimer();
    void                                OnTurnTimer();
    void                                OnFeedback();
private:
    void                                Init();
    void                                GetEncoders();
private:
    class CommImpl;
    boost::shared_ptr<CommImpl>         mImpl;
    std::vector<unsigned char>          mWriteBuffer;
    std::vector<unsigned char>          mReadBuffer;
    
    float                               mEncoder1;
    float                               mEncoder2;
    
    float                               mCurrent1;
    float                               mCurrent2;
    
    float                               mSpeed1;
    float                               mSpeed2;
    
    float                               mEncoderGoal;
    float                               mEncoderGoalForward;
    
    int                                 mVersion;
    
    QTimer*                             mMoveTimer;
    QTimer*                             mTurnTimer;
    QTimer*                             mTimerFeedback;
};

class RobXCommandQueue : public QObject
{
    Q_OBJECT
public:
    explicit RobXCommandQueue(QObject* parent = 0) : QObject(parent) {;}
    virtual ~RobXCommandQueue() {;}
    
    void                AddCommand(const QString& cmd, int param1 = 0, float param2 = 0.0f);
    void                ClearAllCommands();
    
    void                RegisterToRobXModule(RobXControl* rbx);
    
    void                StartExecute();
signals:
    void                AllCommandsDone();
    void                OneCommandExecuted();
    
private slots:
    void                CommandListener();
private:
    bool                CheckCommand(const QString& cmd, int param1, float param2);
    struct Cmd
    {
        std::string     cmdName;
        int             param1;
        int             param2;
    };
    std::queue<Cmd>     mAllCommands;
    RobXControl*        mCtrl;
};

} // end namespace gui
} // end namespace KSRobot

#endif // ROBXCONTROLMODULE_H

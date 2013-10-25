/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2013  <copyright holder> <email>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 * 
 */

#ifndef THREADEDROBOCTRL_H
#define THREADEDROBOCTRL_H

#include <roboctrl/SerialPort.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/atomic.hpp>
#include <boost/function.hpp>
#include <boost/signals2.hpp>

namespace KSRobot
{
namespace roboctrl
{

class ThreadedRoboCtrl
{
public:
    enum MotorCommand
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
    
    
    struct MotorStates
    {
        int                         Encoders[2];
        int                         Voltages[2];
        int                         Currents[2];
        int                         Version;
    };
    
    ThreadedRoboCtrl();
    ~ThreadedRoboCtrl();
    
    void                            SetDevice(const std::string& port);
    
    void                            StartCommThread();
    void                            StopCommThread();
    
    void                            RegisterBeforeStopReciever(boost::function<bool ()> fn);
    void                            RegisterBeforeMoveReciever(boost::function<void ()> fn);
    void                            RegisterBeforeTurnReciever(boost::function<void ()> fn);
    
    boost::signals2::connection     RegisterTimoutReciever(boost::function<void()> fn);
    boost::signals2::connection     RegisterIOErrorReciever(boost::function<void()> fn);
    
    // These accept positive values only.
    void                            StopRobot();
    void                            TurnLeft(int degree, float speed);
    void                            TurnRight(int degree, float speed);
    void                            Forward(int cm, float speed);
    void                            Backward(int cm, float speed);
    
    void                            Turn(int degree, float speed);
    void                            Move(int cm, float speed);
    
private:
    void                            ThreadEntry();
    void                            ThreadTimedCheck();
    bool                            InitMotor();
    void                            ReadEncoders();
    
    void                            StopRobotNoLock();
    
    bool                            CheckForStop();
    
    void                            Write(MotorCommand cmd, unsigned char val1 = 0, unsigned char val2 = 0, unsigned char val3 = 0);
    void                            Read(unsigned char* buffer, size_t num_bytes);
    
    void                            ResetOnTimeout();
    
private:
    boost::mutex                    mCommandMutex;
    boost::thread                   mCommThread;
    boost::atomic<bool>             mContinueExecution;
    
//     boost::atomic<int>              mEncoderGoal;
//     boost::atomic<int>              mEncoderGoalForward;
//     boost::atomic<int>              mCheckEncodersToStop; // 0 = no, 1 = check movement, 2 = check turn.

    // no need to be atomic if only accessed inside a mutex lock
    int                             mEncoderGoal;
    int                             mEncoderGoalForward;
    int                             mCheckEncodersToStop; // 0 = no, 1 = check movement, 2 = check turn.
    
    
    std::string                     mDeviceName;
    SerialPort                      mPort;
    
    boost::function<bool ()>        mBeforeStopRecievers;
    boost::function<void ()>        mBeforeMoveRecievers;
    boost::function<void ()>        mBeforeTurnRecievers;
    
    boost::signals2::signal<void()> mTimeoutRec;
    boost::signals2::signal<void()> mIOErrorRec;
    
    MotorStates                     mMotorState;
};

}
}

#endif // THREADEDROBOCTRL_H

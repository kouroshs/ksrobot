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

#include <roboctrl/ThreadedRoboCtrl.h>

namespace KSRobot
{
namespace roboctrl
{

    typedef unsigned char byte;
    
    struct CommandInfo
    {
        ThreadedRoboCtrl::MotorCommand       command;
        const char*                         name;
        byte                                cmdID;
        byte                                bytesSend;
        byte                                bytesRecv;
    };
    
    static CommandInfo  allCommands[] = {
        {ThreadedRoboCtrl::CMD_GET_SPEED_1,          "CMD_GET_SPEED_1",          0x21, 2, 1}, 
        {ThreadedRoboCtrl::CMD_GET_SPEED_2,          "CMD_GET_SPEED_2",          0x22, 2, 1}, 
        {ThreadedRoboCtrl::CMD_GET_ENCODER_1,        "CMD_GET_ENCODER_1",        0x23, 2, 4}, 
        {ThreadedRoboCtrl::CMD_GET_ENCODER_2,        "CMD_GET_ENCODER_2",        0x24, 2, 4}, 
        {ThreadedRoboCtrl::CMD_GET_ENCODERS,         "CMD_GET_ENCODERS",         0x25, 2, 8}, 
        {ThreadedRoboCtrl::CMD_GET_VOLTS,            "CMD_GET_VOLTS",            0x26, 2, 1}, 
        {ThreadedRoboCtrl::CMD_GET_CURRENT_1,        "CMD_GET_CURRENT_1",        0x27, 2, 1}, 
        {ThreadedRoboCtrl::CMD_GET_CURRENT_2,        "CMD_GET_CURRENT_2",        0x28, 2, 1}, 
        {ThreadedRoboCtrl::CMD_GET_VERSION,          "CMD_GET_VERSION",          0x29, 2, 1}, 
        {ThreadedRoboCtrl::CMD_GET_ACCELERATION,     "CMD_GET_ACCELERATION",     0x2A, 2, 1}, 
        {ThreadedRoboCtrl::CMD_GET_MODE,             "CMD_GET_MODE",             0x2B, 2, 1}, 
        {ThreadedRoboCtrl::CMD_GET_VI,               "CMD_GET_VI",               0x2C, 2, 3}, 
        {ThreadedRoboCtrl::CMD_GET_ERROR,            "CMD_GET_ERROR",            0x2D, 2, 1}, 
        {ThreadedRoboCtrl::CMD_SET_SPEED_1,          "CMD_SET_SPEED_1",          0x31, 3, 0}, 
        {ThreadedRoboCtrl::CMD_SET_SPEED_2,          "CMD_SET_SPEED_2",          0x32, 3, 0}, 
        {ThreadedRoboCtrl::CMD_SET_ACCELERATION,     "CMD_SET_ACCELERATION",     0x33, 3, 0}, 
        {ThreadedRoboCtrl::CMD_SET_MODE,             "CMD_SET_MODE",             0x34, 3, 0}, 
        {ThreadedRoboCtrl::CMD_RESET_ENCODERES,      "CMD_RESET_ENCODERES",      0x35, 2, 0}, 
        {ThreadedRoboCtrl::CMD_DISABLE_REGULATOR,    "CMD_DISABLE_REGULATOR",    0x36, 2, 0}, 
        {ThreadedRoboCtrl::CMD_ENABLE_REGULATOR,     "CMD_ENABLE_REGULATOR",     0x37, 2, 0}, 
        {ThreadedRoboCtrl::CMD_DISABLE_TIMEOUT,      "CMD_DISABLE_TIMEOUT",      0x38, 2, 0}, 
        {ThreadedRoboCtrl::CMD_ENABLE_TIMEOUT,       "CMD_ENABLE_TIMEOUT",       0x39, 2, 0}
    };
    

static bool AlwaysTrueFunction()
{
    return true;
}

static void DefaultVoidFunction()
{
}

ThreadedRoboCtrl::ThreadedRoboCtrl()
{
    mBeforeMoveRecievers = DefaultVoidFunction;
    mBeforeTurnRecievers = DefaultVoidFunction;
    mBeforeStopRecievers = AlwaysTrueFunction;
}

ThreadedRoboCtrl::~ThreadedRoboCtrl()
{
    StopCommThread();
}

void ThreadedRoboCtrl::SetDevice(const std::string& port)
{
    mDeviceName = port;
}

void ThreadedRoboCtrl::StartCommThread()
{
    mContinueExecution.store(true);
    mCommThread = boost::thread(boost::bind(&ThreadedRoboCtrl::ThreadEntry, this));
    boost::this_thread::sleep_for(boost::chrono::seconds(1));
}

void ThreadedRoboCtrl::StopCommThread()
{
    mContinueExecution.store(false);
    if( mCommThread.joinable() )
        mCommThread.join();
}

void ThreadedRoboCtrl::ThreadEntry()
{
    try
    {
        mPort.Open(mDeviceName);
    }
    catch(...)
    {
    }
    
    if( !mPort.IsOpen() )
        return;
    
    if( !InitMotor() )
        return;
    
    while( mContinueExecution )
    {
        ThreadTimedCheck();
        boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
    }
    
    try
    {
        if( mPort.IsOpen() )
            mPort.Close();
    }
    catch(...)
    {
        mIOErrorRec();
        // Do nothing, possibly just report it.
    }
}

bool ThreadedRoboCtrl::CheckForStop()
{
    std::cout << "Encoder 1 = " << mMotorState.Encoders[0] << "  Encoder2 = " << mMotorState.Encoders[1] << std::endl << std::flush;
    try
    {
        if( mCheckEncodersToStop == 0 )
            return false;
        else if( mCheckEncodersToStop == 1 )
        {
            //std::cout << "Goal " << mEncoderGoalForward << std::endl << std::flush;
            if( abs(mMotorState.Encoders[0]) > abs(mEncoderGoalForward) || abs(mMotorState.Encoders[1]) > abs(mEncoderGoalForward) )
            {
                //Write(CMD_RESET_ENCODERES);
                return mBeforeStopRecievers();
            }
            else
                return false;
            // check move.
        }
        else if( mCheckEncodersToStop == 2 )
        {
            if( abs(mMotorState.Encoders[0]) > abs(mEncoderGoal) && abs(mMotorState.Encoders[1]) > abs(mEncoderGoal) )
            {
                //Write(CMD_RESET_ENCODERES);
                return mBeforeStopRecievers();
            }
            else
                return false;
        }
        else
        {
            return false;
        }
    }
    catch(TimeoutException& ex)
    {
        ResetOnTimeout();
        mTimeoutRec();
    }
    catch(...)
    {
        mIOErrorRec();
    }
    return false;
}

void ThreadedRoboCtrl::ThreadTimedCheck()
{
    boost::mutex::scoped_lock lock(mCommandMutex);
    ReadEncoders();
    
    if( CheckForStop() )
    {
        StopRobotNoLock();
        Write(CMD_RESET_ENCODERES);
    }
}

void ThreadedRoboCtrl::StopRobotNoLock()
{
    Write(CMD_SET_SPEED_1, 128);
    Write(CMD_SET_SPEED_2, 128);
}

bool ThreadedRoboCtrl::InitMotor()
{
    boost::mutex::scoped_lock lock(mCommandMutex); // do not let anyone else writing anything.
    try
    {
        boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
        Write(CMD_DISABLE_TIMEOUT);
        boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
        Write(CMD_ENABLE_REGULATOR);
        boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
        Write(CMD_RESET_ENCODERES);
        boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
        return true;
    }
    catch(...)
    {
        mIOErrorRec();
        return false;
    }
}

static inline int SwapBytes(int val)
{
    int b0, b1, b2, b3;
    b0 = (val & 0x000000FF);
    b1 = ((val >> 8) & 0x000000FF);
    b2 = ((val >> 16) & 0x000000FF);
    b3 = ((val >> 24) & 0x000000FF);
    
    return b3 | (b2 << 8) | (b1 << 16) | (b0 << 24);
}

void ThreadedRoboCtrl::ReadEncoders()
{
    try
    {
        int value;
        Write(CMD_GET_ENCODER_1);
        Read((byte*)&value, 4);
        mMotorState.Encoders[0] = SwapBytes(value);
        Write(CMD_GET_ENCODER_2);
        Read((byte*)&value, 4);
        mMotorState.Encoders[1] = SwapBytes(value);
    }
    catch(TimeoutException& ex)
    {
        ResetOnTimeout();
        mTimeoutRec();
    }
    catch(...)
    {
        mIOErrorRec();
    }
}

void ThreadedRoboCtrl::ResetOnTimeout()
{
    mPort.Close();
    mPort.Open(mDeviceName);
}

void ThreadedRoboCtrl::StopRobot()
{
    boost::mutex::scoped_lock lock(mCommandMutex);
    mBeforeStopRecievers(); // their return value has no effect
    StopRobotNoLock();
}

void ThreadedRoboCtrl::Forward(int cm, float speed)
{
    boost::mutex::scoped_lock lock(mCommandMutex);

    int encoder_amount = (-1) * (int)(cm * 24.8);
    int speed_amount = (int)(128 + speed * 10);
    
    Write(CMD_RESET_ENCODERES);
    //ReadEncoders();
    Write(CMD_SET_SPEED_1, (byte)speed_amount);
    Write(CMD_SET_SPEED_2, (byte)speed_amount);

    mEncoderGoalForward = encoder_amount;
    mCheckEncodersToStop = 1;
}

void ThreadedRoboCtrl::Backward(int cm, float speed)
{
    boost::mutex::scoped_lock lock(mCommandMutex);
    
    int encoder_amount = (int)(cm * 24.8);
    int speed_amount = (int)(128 - speed * 10);
    
    Write(CMD_RESET_ENCODERES);
    //ReadEncoders();
    Write(CMD_SET_SPEED_1, speed_amount);
    Write(CMD_SET_SPEED_2, speed_amount);
    
    mEncoderGoalForward = encoder_amount;
    mCheckEncodersToStop = 1;
}

void ThreadedRoboCtrl::Move(int cm, float speed)
{
    if( cm > 0 )
        Forward(cm, speed);
    else
        Backward(-cm, speed);
}

void ThreadedRoboCtrl::TurnLeft(int degree, float speed)
{
    boost::mutex::scoped_lock lock(mCommandMutex);
    
    int encoder_amount = -1 * degree * 11;
    int speed_amount1 = (int)(128 - speed * 10);
    int speed_amount2 = (int)(speed * 10 + 128);
    
    Write(CMD_RESET_ENCODERES);
    Write(CMD_SET_SPEED_1, speed_amount1);
    Write(CMD_SET_SPEED_2, speed_amount2);
    
    mEncoderGoal = encoder_amount;
    mCheckEncodersToStop = 2;
}

void ThreadedRoboCtrl::TurnRight(int degree, float speed)
{
    boost::mutex::scoped_lock lock(mCommandMutex);
    
    int encoder_amount = degree * 11;
    int speed_amount1 = (int)(speed * 10 + 128);
    int speed_amount2 = (int)(128 - speed * 10);
    
    Write(CMD_RESET_ENCODERES);
    Write(CMD_SET_SPEED_1, speed_amount1);
    Write(CMD_SET_SPEED_2, speed_amount2);
    
    mEncoderGoal = encoder_amount;
    mCheckEncodersToStop = 2;
}

void ThreadedRoboCtrl::Turn(int degree, float speed)
{
    if( degree > 0 )
        TurnLeft(degree, speed);
    else
        TurnRight(-degree, speed);
}

void ThreadedRoboCtrl::Read(unsigned char* buffer, size_t num_bytes)
{
    mPort.SetTimeout(boost::chrono::milliseconds(300));
    mPort.ReadTimout(buffer, num_bytes);
}

void ThreadedRoboCtrl::Write(ThreadedRoboCtrl::MotorCommand cmd, unsigned char val1, unsigned char val2, unsigned char val3)
{
    CommandInfo& info = allCommands[(int)cmd];
    unsigned char buffer[5];
    buffer[0] = 0;
    buffer[1] = info.cmdID;
    
    if( info.bytesSend > 2 )
        buffer[2] = val1;
    if( info.bytesSend > 3 )
        buffer[3] = val2;
    if( info.bytesSend > 4 )
        buffer[4] = val3;
    
    mPort.Write(buffer, info.bytesSend);
}

void ThreadedRoboCtrl::RegisterBeforeMoveReciever(boost::function< void() > fn)
{
    mBeforeMoveRecievers = fn;
}

void ThreadedRoboCtrl::RegisterBeforeStopReciever(boost::function< bool() > fn)
{
    mBeforeStopRecievers = fn;
}

void ThreadedRoboCtrl::RegisterBeforeTurnReciever(boost::function< void ()> fn)
{
    mBeforeTurnRecievers = fn;
}

boost::signals2::connection ThreadedRoboCtrl::RegisterIOErrorReciever(boost::function< void() > fn)
{
    return mIOErrorRec.connect(fn);
}

boost::signals2::connection ThreadedRoboCtrl::RegisterTimoutReciever(boost::function< void ()> fn)
{
    return mTimeoutRec.connect(fn);
}

}
}

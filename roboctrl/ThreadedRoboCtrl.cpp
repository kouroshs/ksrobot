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
#include <boost/thread/locks.hpp>
#include <boost/thread/lock_types.hpp>
#include <common/Geometry.h>

using namespace KSRobot::common;

//#define USE_VO_FOR_MOVE

namespace KSRobot
{
namespace roboctrl
{

typedef unsigned char byte;

static inline float AngDiff(float source, float goal)
{
    float diff = goal - source;
    
    if( diff > 180 )
        diff -= 360;
    if( diff < -180 )
        diff = 360 - diff;
    return diff;
}

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

ThreadedRoboCtrl::ThreadedRoboCtrl() : mAnyCommandExecuting(false)
{
    mUseVO = false;
    mCurrentPosition = Eigen::Vector2f(0, 0);
    mYaw = 0;
}

ThreadedRoboCtrl::~ThreadedRoboCtrl()
{
    StopCommThread();
}

void ThreadedRoboCtrl::SetDevice(const std::string& port, const std::string& compass_device)
{
    mDeviceName = port;
    mCompassDeviceName = compass_device;
    mUseCompass = false;
    if( mCompassDeviceName != "" )
        mUseCompass = true;
}

void ThreadedRoboCtrl::StartCommThread()
{
    mContinueExecution.store(true);
    mCommThread = boost::thread(boost::bind(&ThreadedRoboCtrl::ThreadEntry, this));
    if( mUseCompass )
        mCompassThread = boost::thread(boost::bind(&ThreadedRoboCtrl::CompassUpdateThreadEntry, this));
    boost::this_thread::sleep_for(boost::chrono::seconds(1));
}

void ThreadedRoboCtrl::StopCommThread()
{
    mContinueExecution.store(false);
    if( mCommThread.joinable() )
        mCommThread.join();
    if( mCompassThread.joinable() )
        mCompassThread.join();
}

void ThreadedRoboCtrl::CompassUpdateThreadEntry()
{
    mCompass.Open(mCompassDeviceName, 9600, 8,
                  SerialPort::Parity(SerialPort::Parity::none),
                  SerialPort::StopBits(SerialPort::StopBits::two),
                  SerialPort::FlowControl(SerialPort::FlowControl::none));
    
    if( !mCompass.IsOpen() )
        return;
    
    mCompass.SetTimeout(boost::chrono::milliseconds(100));
    // no kind of initialization is needed fortunately
    unsigned char write_val = 0x13;
    unsigned char buffer[2];

    mCompass.Write(&write_val, 1);
    mCompass.ReadTimout(buffer, 2);
    unsigned int val = ((unsigned int)buffer[0] << 8) + (unsigned int)buffer[1];
    mCompassStartAngle = (common::AngDeg)val / 10.0f;
    
    while( mContinueExecution )
    {
        mCompass.Write(&write_val, 1);
        mCompass.ReadTimout(buffer, 2);
        val = ((unsigned int)buffer[0] << 8) + (unsigned int)buffer[1];
        
        mCompassAngle.store((float)val / 10.0f);
        
        mYaw = common::AngleDifferenceDegrees(mCompassAngle, mCompassStartAngle);
        
        boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
    }
}

void ThreadedRoboCtrl::ThreadEntry()
{
    mPort.Open(mDeviceName);
    
    if( !mPort.IsOpen() )
        return;

    if( !InitMotor() )
        return;
    
    while( mContinueExecution )
    {
        ThreadTimedCheck();
        boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
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

bool ThreadedRoboCtrl::ContinuousCommand(const RobotCommand::Command& rc) const
{
    return (rc != RobotCommand::CMD_EMPTY) && (rc != RobotCommand::CMD_STOP);
}

void ThreadedRoboCtrl::NextCommand()
{
    const RobotCommand curr_cmd = mCommandQueue.front();
    mCommandQueue.pop();
    if( mCommandQueue.empty() )
    {
        std::cout << "NEXT COMMAND EMPTY\n" << std::flush;
        // command queue is empty, stop
        mSuddenMoveRecievers();
        StopRobot();
        mCommandsEvent.Fire(); // queue is finally empty, so we should fire this event.
    }
    else if( mCommandQueue.front().GetCommand() == curr_cmd.GetCommand() && 
             mCommandQueue.front().GetSpeed() == curr_cmd.GetSpeed() )
    {
        std::cout << "NEXT COMMAND No sudden move\n" << std::flush;
        // no sudden move, just execute next command
        ExecCommand(mCommandQueue.front());
        mAnyCommandExecuting = ContinuousCommand(mCommandQueue.front().GetCommand());
    }
    else
    {
        // again, sudden movement
        mSuddenMoveRecievers();
        ExecCommand(mCommandQueue.front());
        mAnyCommandExecuting = ContinuousCommand(mCommandQueue.front().GetCommand());
        //StopRobot();
    }
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
        
        std::cout << "Encoder1 = " << mMotorState.Encoders[0] << "  Encoder 2 = " << mMotorState.Encoders[1] << std::endl << std::flush;
    }
    catch(TimeoutException& ex)
    {
        std::cout << "(ThreadedRoboCtrl::ReadEncoders) Timeout exception.\n" << std::flush;
        ResetOnTimeout();
        mTimeoutRec();
    }
    catch(...)
    {
        std::cout << "(ThreadedRoboCtrl::ReadEncoders) unknown exception.\n" << std::flush;
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
    boost::unique_lock<boost::mutex> lock(mCommandMutex);
    Write(CMD_SET_SPEED_1, 128);
    Write(CMD_SET_SPEED_2, 128);
}

void ThreadedRoboCtrl::Move(int cm, float speed)
{
    boost::unique_lock<boost::mutex> lock(mCommandMutex);
    
    int sign = sgn(cm);
    int encoder_amount = -sign * (int)(cm * 24.8);
    int speed_amount = (int)(128 + sign * speed * 10);

    if( mUseCompass )
        mMoveMaintainAngle = mCompassAngle.load();
    
    mPosMoveStart = mCurrentPosition;
    
    Write(CMD_RESET_ENCODERES);
    Write(CMD_SET_SPEED_1, (byte)speed_amount);
    Write(CMD_SET_SPEED_2, (byte)speed_amount);

    if( mUseVO )
    {
        boost::unique_lock<boost::mutex> lock(mVO_Lock);
        mVO_MoveValueMeters = cm * 0.01;
        mVO_StartPosition = mVO_CurrentPosition;
        mVO_StartYaw = mVO_CurrentYaw;
    }
    mEncoderGoalForward = encoder_amount;
}

void ThreadedRoboCtrl::Turn(int degree, float speed)
{
    boost::unique_lock<boost::mutex> lock(mCommandMutex);
    if( mUseCompass )
    {
        //TODO: What should be the goal? should I add or subtract?
        //FIXME: For now assume adding
        
        mTurnStartAngle = mCompassAngle.load();
        mTurnGoalAngle = fmod(mTurnStartAngle - degree, 360);
        
        TurnWithSignAndSpeed(sgn(degree), speed);
    }
    else
    {
        int encoder_amount = -degree * 11;
        Write(CMD_RESET_ENCODERES);
        TurnWithSignAndSpeed(sgn(degree), speed);
        mEncoderGoal = encoder_amount;
    }
}

void ThreadedRoboCtrl::TurnTo(int degree, float speed)
{
    if( mUseCompass )
    {
        mTurnStartAngle = mCompassAngle.load();
        mTurnGoalAngle = degree;
        TurnWithSignAndSpeed(sgn(AngDiff(mTurnGoalAngle, mTurnStartAngle)), speed);
    }
}


void ThreadedRoboCtrl::TurnWithSignAndSpeed(int sign, float speed)
{
    //std::cout << "SIGN " << sign << "  SPEED  " << speed << std::endl << std::flush;
    int motor1_speed = 128 - sign * 10 * speed;
    int motor2_speed = 128 + sign * 10 * speed;
    
    Write(CMD_SET_SPEED_1, motor1_speed);
    Write(CMD_SET_SPEED_2, motor2_speed);
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

void ThreadedRoboCtrl::TestCommand(int i, float degree)
{
    boost::unique_lock<boost::mutex> lock(mQueueLock);
    mTurnGoalAngle = degree;
    
    TurnWithSignAndSpeed(sgn(degree), 1);
}

void ThreadedRoboCtrl::ExecCommand(const RobotCommand& rc)
{
    if( rc.mCmd == RobotCommand::CMD_MOVE )
        Move(rc.iVal, rc.fVal);
    else if( rc.mCmd == RobotCommand::CMD_TURN )
        Turn(rc.iVal, rc.fVal);
    else if( rc.mCmd == RobotCommand::CMD_TURN_TO )
        TurnTo(rc.iVal, rc.fVal);
    else if( rc.mCmd == RobotCommand::CMD_STOP )
        StopRobot();
    else
        TestCommand(rc.iVal, rc.fVal);
}

boost::signals2::connection ThreadedRoboCtrl::RegisterSuddenMoveReceiver(boost::function< void ()> fn)
{
    return mSuddenMoveRecievers.connect(fn);
}

boost::signals2::connection ThreadedRoboCtrl::RegisterIOErrorReciever(boost::function< void() > fn)
{
    return mIOErrorRec.connect(fn);
}

boost::signals2::connection ThreadedRoboCtrl::RegisterTimoutReciever(boost::function< void ()> fn)
{
    return mTimeoutRec.connect(fn);
}

void ThreadedRoboCtrl::PutCommandToQueue(const RobotCommand& rc)
{
    boost::unique_lock<boost::mutex> lock(mQueueLock);
    mCommandsEvent.Reset();
    
    if( rc.mCmd == RobotCommand::CMD_MOVE && rc.iVal > 100 )
    {
        int v = rc.iVal;
        int times = rc.iVal / 100;
        for(int i = 0; i < times; i ++)
            mCommandQueue.push(RobotCommand::Move(100, rc.fVal));
        int rem = rc.iVal % 100;
        if( rem > 0 )
            mCommandQueue.push(RobotCommand::Move(rem, rc.fVal));
    }
    else
        mCommandQueue.push(rc);
}

void ThreadedRoboCtrl::WaitForAllCommands()
{
    mCommandsEvent.Wait();
    mCommandsEvent.Reset();
}

void ThreadedRoboCtrl::ClearCommandQueue(bool bContinueExecutingCurrentCommand)
{
    boost::unique_lock<boost::mutex> lock(mQueueLock);
    
    if( bContinueExecutingCurrentCommand )
    {
        if( !mCommandQueue.empty() && mAnyCommandExecuting )
        {
            RobotCommand rc = mCommandQueue.front();
            mCommandQueue = std::queue<RobotCommand>();
            mCommandQueue.push(rc);
        }
    }
    else
    {
        if( mAnyCommandExecuting )
            ExecCommand(RobotCommand::Stop());
        mAnyCommandExecuting = false;
        mCommandQueue = std::queue<RobotCommand>();
    }
}

void ThreadedRoboCtrl::ThreadTimedCheck()
{
    try
    {
        boost::mutex::scoped_lock lock(mQueueLock);

        ReadEncoders();
        
        if( !mAnyCommandExecuting && !mCommandQueue.empty() )
        {
            ExecCommand(mCommandQueue.front());
            mAnyCommandExecuting = ContinuousCommand(mCommandQueue.front().GetCommand());
            return;
        }
        
        
        if( mCommandQueue.empty() )
        {
            return;
        }
        switch( mCommandQueue.front().GetCommand() )
        {
            case RobotCommand::CMD_STOP:
                // no call to sudden move recievers, since the only way this is called is when we have already called the function
                // in previous cycle.
                StopRobot();
                mAnyCommandExecuting = false;
                mCommandQueue.pop();
                break;
            case RobotCommand::CMD_MOVE:
                if( mUseVO )
                {
                    boost::unique_lock<boost::mutex> lock(mVO_Lock);
                    std::cout << "CURRENT DIST TO GOAL : " << (mVO_CurrentPosition - mVO_StartPosition).norm() << std::endl << std::flush;
                    if( (mVO_CurrentPosition - mVO_StartPosition).norm() >= mVO_MoveValueMeters )
                        NextCommand();
                }
                else
                {
                    float dist = fabs(((float)mMotorState.Encoders[0]) / 24.8f);
                    float angle = mCompassAngle.load();
                    
                    mCurrentPosition = mPosMoveStart + Eigen::Vector2f(dist * cos(angle), dist * sin(angle));
                    
                    if( abs(mMotorState.Encoders[0]) > abs(mEncoderGoalForward) || abs(mMotorState.Encoders[1]) > abs(mEncoderGoalForward) )
                        NextCommand();
                }
                break;
            case RobotCommand::CMD_TURN_TO:
            case RobotCommand::CMD_TURN:
                if( mUseCompass )
                {
                    float error = AngDiff(mTurnGoalAngle, mCompassAngle.load());
                    const float kp = 0.03f;

                    if( fabs(error) > 5 )
                    {
                        float resulting_speed = error * kp;
                        resulting_speed = clamp(resulting_speed, -1.0f, 1.0f);
                        std::cout << "ERROR " << error  << "  Current Ang: " << mCompassAngle << "  GOAL : " << mTurnGoalAngle << std::endl;
                        TurnWithSignAndSpeed(sgn(resulting_speed), fabs(resulting_speed));
                    }
                    else
                    {
                        NextCommand();
                    }
                }
                else
                {
                    if( abs(mMotorState.Encoders[0]) > abs(mEncoderGoal) && abs(mMotorState.Encoders[1]) > abs(mEncoderGoal) )
                        NextCommand();
                }
                break;
            case RobotCommand::CMD_EMPTY:
                if( fabs(mCompassAngle - mTurnGoalAngle) < 3 )
                {
                    //StopRobot();
                    NextCommand();
                }
                break;
        }
    }
    catch(TimeoutException& ex)
    {
        std::cout << "TimeoutException\n" << std::flush;
        ResetOnTimeout();
        mTimeoutRec();
    }
    catch(...)
    {
        std::cout << "Unknown error\n" << std::flush;
        mIOErrorRec();
    }
}



}
}


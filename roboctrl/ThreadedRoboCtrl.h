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

#include <common/Event.h>
#include <common/Geometry.h>
#include <roboctrl/SerialPort.h>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/atomic.hpp>
#include <boost/function.hpp>
#include <boost/signals2.hpp>
#include <queue>

namespace KSRobot
{
namespace roboctrl
{
class ThreadedRoboCtrl;
class RobotCommand
{
public:
    enum Command
    {
        CMD_MOVE,
        CMD_TURN,
        CMD_TURN_TO,
        CMD_STOP,
        CMD_EMPTY
    };
    
    RobotCommand() : mCmd(CMD_EMPTY) {;}
    RobotCommand(Command cmd, int ival, float fval) : mCmd(cmd), iVal(ival), fVal(fval) {;}
    
    Command GetCommand() const { return mCmd; }
    float   GetSpeed() const { return fVal; }
    int     GetAmount() const { return iVal; }
    
    static RobotCommand Move(int cm, float speed = 1.0f) { return RobotCommand(CMD_MOVE, cm, speed); }
    static RobotCommand Turn(int degree, float speed = 1.0f) { return RobotCommand(CMD_TURN, degree, speed); }
    static RobotCommand TurnTo(int degree, float speed = 1.0f) { return RobotCommand(CMD_TURN_TO, degree, speed); }
    static RobotCommand Stop() { return RobotCommand(CMD_STOP, 0, 0); }
private:
    friend class ThreadedRoboCtrl;
    
    Command         mCmd;
    int             iVal;
    float           fVal;
    
};

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

    typedef ThreadedRoboCtrl                            this_type;
    typedef boost::shared_ptr<ThreadedRoboCtrl>         Ptr;
    typedef boost::shared_ptr<const ThreadedRoboCtrl>   ConstPtr;
    
    ThreadedRoboCtrl();
    ~ThreadedRoboCtrl();
    
    void                            SetDevice(const std::string& robx_motor_device, const std::string& compass_device = "");
    
    void                            StartCommThread();
    void                            StopCommThread();
    
    boost::signals2::connection     RegisterSuddenMoveReceiver(boost::function<void()> fn);
    boost::signals2::connection     RegisterTimoutReciever(boost::function<void()> fn);
    boost::signals2::connection     RegisterIOErrorReciever(boost::function<void()> fn);
    
    // These accept positive values only.
    
    inline float                    GetCompassAngle() const { return mCompassAngle.load(); }
    
    void                            PutCommandToQueue(const RobotCommand& rc);
    void                            ClearCommandQueue(bool bContinueExecutingCurrentCommand = true);
    void                            WaitForAllCommands();
    void                            TestCommand(int , float);
private:
    void                            ThreadEntry();
    void                            ThreadTimedCheck();
    bool                            InitMotor();
    void                            ReadEncoders();
    void                            NextCommand();
    
    void                            Write(MotorCommand cmd, unsigned char val1 = 0, unsigned char val2 = 0, unsigned char val3 = 0);
    void                            Read(unsigned char* buffer, size_t num_bytes);
    
    void                            ResetOnTimeout();

    void                            StopRobot();
    void                            Turn(int degree, float speed);
    void                            TurnTo(int degree, float speed);
    void                            Move(int cm, float speed);

    void                            TurnWithSignAndSpeed(int sign, float speed);
    
    void                            ExecCommand(const RobotCommand& rc);
    bool                            ContinuousCommand(const RobotCommand::Command& rc) const;
    
    
    void                            CompassUpdateThreadEntry();
private:
    struct MotorStates
    {
        int                         Encoders[2];
        int                         Voltages[2];
        int                         Currents[2];
        int                         Version;
    };
    
    boost::mutex                    mCommandMutex;
    boost::thread                   mCommThread;
    boost::atomic<bool>             mContinueExecution;
    
    // no need to be atomic if only accessed inside a mutex lock
    int                             mEncoderGoal;
    int                             mEncoderGoalForward;
    int                             mCheckEncodersToStop; // 0 = no, 1 = check movement, 2 = check turn.
    
    bool                            mAnyCommandExecuting;
    
    std::string                     mDeviceName;
    SerialPort                      mPort;
    
    boost::signals2::signal<void()> mSuddenMoveRecievers;
    
    boost::mutex                    mQueueLock;
    std::queue<RobotCommand>        mCommandQueue;
    common::Event                   mCommandsEvent;
    
    boost::signals2::signal<void()> mTimeoutRec;
    boost::signals2::signal<void()> mIOErrorRec;
    
    MotorStates                     mMotorState;
    
    std::string                     mCompassDeviceName;
    SerialPort                      mCompass;
    boost::thread                   mCompassThread;
    bool                            mUseCompass;
    boost::atomic<float>            mCompassAngle;
    common::AngDeg                  mCompassStartAngle;
    float                           mTurnStartAngle;
    float                           mTurnGoalAngle;
    float                           mMoveMaintainAngle;
    
public:
    
    
    Eigen::Vector2f                 mPosMoveStart;
    Eigen::Vector2f                 mCurrentPosition;
    common::AngDeg                  mYaw;
    
    boost::mutex                    mVO_Lock;
    bool                            mUseVO;
    Eigen::Vector2f                 mVO_StartPosition;
    float                           mVO_StartYaw;
    Eigen::Vector2f                 mVO_CurrentPosition;
    float                           mVO_CurrentYaw;
    float                           mVO_MoveValueMeters;
};

}
}

#endif // THREADEDROBOCTRL_H

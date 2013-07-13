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

#include <roboctrl/RobXControl.h>

#include <common/Defenitions.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <math.h>
#include <QTimer>

#include <iostream>
#include <iomanip>

using std::cout;
using std::endl;
using std::flush;
using std::hex;

namespace KSRobot
{
namespace roboctrl
{

typedef unsigned char byte;
    
struct RobXCommandInfo
{
    RobXControl::RobXMotorCommand       command;
    const char*                         name;
    byte                                cmdID;
    byte                                bytesSend;
    byte                                bytesRecv;
};

static RobXCommandInfo  allCommands[] = {
    {RobXControl::CMD_GET_SPEED_1,          "CMD_GET_SPEED_1",          0x21, 2, 1}, 
    {RobXControl::CMD_GET_SPEED_2,          "CMD_GET_SPEED_2",          0x22, 2, 1}, 
    {RobXControl::CMD_GET_ENCODER_1,        "CMD_GET_ENCODER_1",        0x23, 2, 4}, 
    {RobXControl::CMD_GET_ENCODER_2,        "CMD_GET_ENCODER_2",        0x24, 2, 4}, 
    {RobXControl::CMD_GET_ENCODERS,         "CMD_GET_ENCODERS",         0x25, 2, 8}, 
    {RobXControl::CMD_GET_VOLTS,            "CMD_GET_VOLTS",            0x26, 2, 1}, 
    {RobXControl::CMD_GET_CURRENT_1,        "CMD_GET_CURRENT_1",        0x27, 2, 1}, 
    {RobXControl::CMD_GET_CURRENT_2,        "CMD_GET_CURRENT_2",        0x28, 2, 1}, 
    {RobXControl::CMD_GET_VERSION,          "CMD_GET_VERSION",          0x29, 2, 1}, 
    {RobXControl::CMD_GET_ACCELERATION,     "CMD_GET_ACCELERATION",     0x2A, 2, 1}, 
    {RobXControl::CMD_GET_MODE,             "CMD_GET_MODE",             0x2B, 2, 1}, 
    {RobXControl::CMD_GET_VI,               "CMD_GET_VI",               0x2C, 2, 3}, 
    {RobXControl::CMD_GET_ERROR,            "CMD_GET_ERROR",            0x2D, 2, 1}, 
    {RobXControl::CMD_SET_SPEED_1,          "CMD_SET_SPEED_1",          0x31, 3, 0}, 
    {RobXControl::CMD_SET_SPEED_2,          "CMD_SET_SPEED_2",          0x32, 3, 0}, 
    {RobXControl::CMD_SET_ACCELERATION,     "CMD_SET_ACCELERATION",     0x33, 3, 0}, 
    {RobXControl::CMD_SET_MODE,             "CMD_SET_MODE",             0x34, 3, 0}, 
    {RobXControl::CMD_RESET_ENCODERES,      "CMD_RESET_ENCODERES",      0x35, 2, 0}, 
    {RobXControl::CMD_DISABLE_REGULATOR,    "CMD_DISABLE_REGULATOR",    0x36, 2, 0}, 
    {RobXControl::CMD_ENABLE_REGULATOR,     "CMD_ENABLE_REGULATOR",     0x37, 2, 0}, 
    {RobXControl::CMD_DISABLE_TIMEOUT,      "CMD_DISABLE_TIMEOUT",      0x38, 2, 0}, 
    {RobXControl::CMD_ENABLE_TIMEOUT,       "CMD_ENABLE_TIMEOUT",       0x39, 2, 0}
};

#ifndef NDEBUG
    #define SET_CURR_FN         mCallerName.push(__PRETTY_FUNCTION__);
    #define RESET_CURR_FN       mCallerName.pop();
#else
    #define SET_CURR_FN
    #define RESET_CURR_FN
#endif

RobXControl::RobXControl(QObject* parent): QObject(parent), mUseReadTimeout(true), mReadTimeoutMillisecs(500), mReadSingleByte(true)
{
    mMoveTimer = new QTimer(this);
    mTurnTimer = new QTimer(this);
    mTimerFeedback = new QTimer(this);
    
    mMoveTimer->setInterval(100);
    mTurnTimer->setInterval(100);
    mTimerFeedback->setInterval(200);
    
    connect(mMoveTimer, SIGNAL(timeout()), this, SLOT(OnMoveTimer()));
    connect(mTurnTimer, SIGNAL(timeout()), this, SLOT(OnTurnTimer()));
    connect(mTimerFeedback, SIGNAL(timeout()), this, SLOT(OnFeedback()));
}

RobXControl::~RobXControl()
{
}

void RobXControl::ReadSettings(common::ProgramOptions::Ptr po)
{
    mUseReadTimeout = po->GetBool("UseReadTimeout", false);
    mReadTimeoutMillisecs = po->GetInt("ReadTimeoutMillisecs", 100);
    mReadSingleByte = po->GetBool("ReadSingleByte", false);
}

bool RobXControl::IsOpen()
{
    return mPort.IsOpen();
}

void RobXControl::Open(const QString& device)
{
    mDeviceName = device.toStdString();
    mPort.Open(mDeviceName, 9600);
    Init();
}

void RobXControl::Close()
{
    mPort.Close();
}

void RobXControl::Read(unsigned char* buffer, size_t numBytes)
{
    //std::cout << "(RobXControl::Read) start " << numBytes << std::flush;
    if( mUseReadTimeout )
    {
        mPort.SetTimeout(boost::chrono::milliseconds(mReadTimeoutMillisecs));
        mPort.ReadTimout(buffer, numBytes);
    }
    else
    {
        mPort.ReadBlocking(buffer, numBytes, mReadSingleByte);
    }
    
    //std::cout << " done\n" << std::flush;
}

void RobXControl::Read(std::vector<unsigned char>& buffer, size_t numBytes)
{
    buffer.clear();
    buffer.resize(numBytes);
    Read(buffer.data(), numBytes);
}

void RobXControl::Write(RobXMotorCommand cmd, unsigned char val1, unsigned char val2, unsigned char val3)
{
    RobXCommandInfo& info = allCommands[(int)cmd];
    assert(info.command == cmd);
    mWriteBuffer.clear();
    mWriteBuffer.push_back(0x00);
    mWriteBuffer.push_back(info.cmdID);
    
    if( info.bytesSend > 2 )
        mWriteBuffer.push_back(val1);
    if( info.bytesSend > 3 )
        mWriteBuffer.push_back(val2);
    if( info.bytesSend > 4 )
        mWriteBuffer.push_back(val3);
    
//     std::cout << "(RobXControl::Write) " << info.name << endl << flush;
//    boost::asio::write(mImpl->Serial(), boost::asio::buffer(mWriteBuffer.data(), info.bytesSend));
    mPort.Write(mWriteBuffer.data(), info.bytesSend);
}

void RobXControl::Read(size_t numBytes)
{
    if( numBytes == 0 )
        return;
    Read(mReadBuffer, numBytes);
}

void RobXControl::OnMoveTimer()
{
    if( abs(mEncoder1) > abs(mEncoderGoalForward) || abs(mEncoder2) > abs(mEncoderGoalForward) )
    {
        std::cout << "(RobXControl::OnMoveTimer) Move command done\n" << std::flush;
        SET_CURR_FN;
        Stop();
        mMoveTimer->stop();
        Write(CMD_RESET_ENCODERES);
        RESET_CURR_FN;
        emit OnCommandDone();
    }
}

void RobXControl::OnTurnTimer()
{
    if( abs(mEncoder1) > abs(mEncoderGoal) )
    {
        if( abs(mEncoder2) > abs(mEncoderGoal) )
        {
            SET_CURR_FN;
            Stop();
            mTurnTimer->stop();
            Write(CMD_RESET_ENCODERES);
            RESET_CURR_FN;
            emit OnCommandDone();
        }
    }
}

void RobXControl::Stop()
{
    SET_CURR_FN;
    Write(CMD_SET_SPEED_1, 128);
    Write(CMD_SET_SPEED_2, 128);
    RESET_CURR_FN;
}

void RobXControl::Forward(int cm, float speed_index)
{
    int encoder_amount = (int)(cm * 24.8);
    int speed_amount = (int)(128 - speed_index * 10);
    //int speed_amount2 = (int)(128 - speed_index * 10);
    
    SET_CURR_FN;
    
    Write(CMD_RESET_ENCODERES);
    GetEncoders();
    
    Write(CMD_SET_SPEED_1, (byte)speed_amount);
    Write(CMD_SET_SPEED_2, (byte)speed_amount);
    
    RESET_CURR_FN;
    
    std::cout << "(RobXControl::Forward) Amount = " << speed_amount << " ToByte " << (int)(byte)speed_amount << std::endl << std::flush;
    std::cout << "\t Encoder must = " << encoder_amount << endl << flush;
    
    mEncoderGoalForward = encoder_amount;
    mMoveTimer->start();
}

void RobXControl::Backward(int cm, float speed_index)
{
    int encoder_amount = (-1) * (int)(cm * 24.8);
    int speed_amount = (int)(128 + speed_index * 10);
    //int speed_amoufnt2 = (int)(128 + speed_index * 10);

    SET_CURR_FN;
    
    Write(CMD_RESET_ENCODERES);
    GetEncoders();
    
    Write(CMD_SET_SPEED_1, speed_amount);
    Write(CMD_SET_SPEED_2, speed_amount);
    
    RESET_CURR_FN;
    
    std::cout << "(RobXControl::Backward) Amount = " << speed_amount << " ToByte " << (int)(byte)speed_amount << std::endl << std::flush;
    
    mEncoderGoalForward = encoder_amount;
    mMoveTimer->start();
}

void RobXControl::TurnLeft(int degree, float speed_index)
{
    int encoder_amount = degree * 11;
    int speed_amount1 = (int)(speed_index * 10 + 128);
    int speed_amount2 = (int)(128 - speed_index * 10);
    
    SET_CURR_FN;
    
    Write(CMD_SET_SPEED_1, speed_amount1);
    Write(CMD_SET_SPEED_2, speed_amount2);
    
    RESET_CURR_FN;
    
    mEncoderGoal = encoder_amount;
    mTurnTimer->start();
}

void RobXControl::TurnRight(int degree, float speed_index)
{
    int encoder_amount = -1 * degree * 11;
    int speed_amount1 = (int)(128 - speed_index * 10);
    int speed_amount2 = (int)(speed_index * 10 + 128);
    
    SET_CURR_FN;
    
    Write(CMD_SET_SPEED_1, speed_amount1);
    Write(CMD_SET_SPEED_2, speed_amount2);
    
    RESET_CURR_FN;
    
    mEncoderGoal = encoder_amount;
    mTurnTimer->start();
}

void RobXControl::EnableAutoregulator(bool enable)
{
    SET_CURR_FN;
    if( enable )
        Write(CMD_ENABLE_REGULATOR);
    else
        Write(CMD_DISABLE_REGULATOR);
    RESET_CURR_FN;
}

void RobXControl::EnableTimeout(bool enable)
{
    SET_CURR_FN;
    if( enable )
        Write(CMD_ENABLE_TIMEOUT);
    else
        Write(CMD_DISABLE_TIMEOUT);
    RESET_CURR_FN;
}

static int SwapBytes(int val)
{
    int b0, b1, b2, b3;
    b0 = (val & 0x000000FF);
    b1 = ((val >> 8) & 0x000000FF);
    b2 = ((val >> 16) & 0x000000FF);
    b3 = ((val >> 24) & 0x000000FF);
    
    return b3 | (b2 << 8) | (b1 << 16) | (b0 << 24);
}

void RobXControl::GetEncoders()
{
    try
    {
        int value = 0;
//         unsigned char c;
//        std::cout << "(RobXControl::GetEncoders) Start get ... " << std::flush;
        SET_CURR_FN;
        
        Write(CMD_GET_ENCODER_1);
        Read((unsigned char*)&value, 4);
        mEncoder1 = SwapBytes(value);
        
        Write(CMD_GET_ENCODER_2);
        Read((unsigned char*)&value, 4);
        mEncoder2 = SwapBytes(value);

        RESET_CURR_FN;
//        std::cout << " Encoder1 = " <<  mEncoder1 << " Encoder2 = " << mEncoder2 << std::endl << std::flush;
    }
    catch(std::exception& ex)
    {
        std::cout << "(RobXControl::GetEncoders) Exception occured: " << ex.what() << std::endl << std::flush;
        //RESET Serial Port
        mPort.Close();
        mPort.Open(mDeviceName, 9600);
    }
    catch(...)
    {
        std::cout << "(RobXControl::GetEncoders) Unknown exception occured.\n" << std::flush;
    }
}

int RobXControl::GetVoltage()
{
    Write(CMD_GET_VOLTS);
    Read(1);
    return (int)mReadBuffer[0];
}

int RobXControl::GetVersion()
{
    Write(CMD_GET_VERSION);
    Read(1);
    return (int)mReadBuffer[0];
}

int RobXControl::GetCurrent(int index)
{
    if(index == 1)
        Write(CMD_GET_CURRENT_1);
    else if(index == 2)
        Write(CMD_GET_CURRENT_2);
    else
    {
        std::cout << "(RobXControl::GetCurrent) Invalid index " << index << " was provided.\n";
        return 0;
    }
    Read(1);
    return (int)mReadBuffer[0] * 100;
}

#define SLEEP(x) boost::this_thread::sleep_for(boost::chrono::milliseconds(x))

void RobXControl::Init()
{
    SLEEP(100);
    EnableTimeout(false);
    SLEEP(100);
    EnableAutoregulator(true);
    SLEEP(100);
    Write(CMD_RESET_ENCODERES);
    SLEEP(500);
    mTimerFeedback->start();
}

void RobXControl::OnFeedback()
{
//     static int i = 0;
//     std::cout << "(ONFeedback) before encoderes ... " << i++ << std::flush;
    GetEncoders();
//     std::cout << " done feedback \n " << std::flush;
}

int RobXControl::GetEncoderValue(int index) const
{
    if( index == 1 )
        return mEncoder1;
    else if( index == 2 )
        return mEncoder2;
    else
    {
        throw std::runtime_error("(RobXControl::GetEncoderValue) invalid encoder index.");
    }
}





void RobXCommandQueue::AddCommand(const QString& cmd, int param1, float param2)
{
    if( !CheckCommand(cmd, param1, param2) )
    {
        std::cout << "(RobXCommandQueue::AddCommand) Invalid command ignored : <" << cmd.toStdString() << ">(" << 
                    param1 << ", " << param2 << ")\n" << std::flush;
        return;
    }
    Cmd c;
    c.cmdName = cmd.toLower().toStdString();
    c.param1 = param1;
    c.param2 = param2;
    mAllCommands.push(c);
}

bool RobXCommandQueue::CheckCommand(const QString& cmd, int param1, float param2)
{
    (void)param1;
    (void)param2;
    QString str = cmd.toLower();
    return (str == "forward" || str == "backward" || str == "turnright" || str == "turnleft");
}

void RobXCommandQueue::ClearAllCommands()
{
    while( !mAllCommands.empty() )
        mAllCommands.pop();
}

void RobXCommandQueue::RegisterToRobXModule(RobXControl* rbx)
{
    connect(rbx, SIGNAL(OnCommandDone()), this, SLOT(CommandListener()));
    mCtrl = rbx;
}

void RobXCommandQueue::CommandListener()
{
    if( mAllCommands.size() == 0 )
    {
        emit AllCommandsDone();
    }
    else
    {
        emit OneCommandExecuted();
        StartExecute();
    }
}

void RobXCommandQueue::StartExecute()
{
    Cmd c = mAllCommands.front();
    mAllCommands.pop();
    
    if( c.cmdName == "forward" )
        emit mCtrl->Forward(c.param1, c.param2);
    else if( c.cmdName == "backward" )
        emit mCtrl->Backward(c.param1, c.param2);
    else if( c.cmdName == "turnleft" )
        emit mCtrl->TurnLeft(c.param1, c.param2);
    else if( c.cmdName == "turnright" )
        emit mCtrl->TurnRight(c.param1, c.param2);
    else // STOP Command, any invalid commands that got to here will result in robot stop.
        emit mCtrl->Stop();
}

} // end namespace gui
} // end namespace KSRobot


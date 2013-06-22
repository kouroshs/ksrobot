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

#include <gui/RobXControl.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <math.h>
#include <QTimer>


namespace KSRobot
{
namespace gui
{

typedef unsigned char byte;
    
struct RobXCommandInfo
{
    RobXControl::RobXMotorCommand       command;
    byte                                cmdID;
    byte                                bytesSend;
    byte                                bytesRecv;
};

static RobXCommandInfo  allCommands[] = {
    {RobXControl::CMD_GET_SPEED_1, 0x21,  2,  1}, 
    {RobXControl::CMD_GET_SPEED_2, 0x22,  2,  1}, 
    {RobXControl::CMD_GET_ENCODER_1, 0x23,  2, 4}, 
    {RobXControl::CMD_GET_ENCODER_2,  0x24,  2,  4}, 
    {RobXControl::CMD_GET_ENCODERS,  0x25,  2,  8}, 
    {RobXControl::CMD_GET_VOLTS,  0x26,  2, 1}, 
    {RobXControl::CMD_GET_CURRENT_1,  0x27,  2, 1}, 
    {RobXControl::CMD_GET_CURRENT_2,  0x28,  2, 1}, 
    {RobXControl::CMD_GET_VERSION,  0x29,  2,  1}, 
    {RobXControl::CMD_GET_ACCELERATION,  0x2A,  2, 1}, 
    {RobXControl::CMD_GET_MODE,  0x2B,  2,  1}, 
    {RobXControl::CMD_GET_VI, 0x2C,  2, 3}, 
    {RobXControl::CMD_GET_ERROR,  0x2D,  2, 1}, 
    {RobXControl::CMD_SET_SPEED_1,  0x31,  3,  0}, 
    {RobXControl::CMD_SET_SPEED_2,  0x32,  3, 0}, 
    {RobXControl::CMD_SET_ACCELERATION, 0x33, 3, 0}, 
    {RobXControl::CMD_SET_MODE, 0x34, 3, 0}, 
    {RobXControl::CMD_RESET_ENCODERES,  0x35,  2, 0}, 
    {RobXControl::CMD_DISABLE_REGULATOR,  0x36,  2, 0}, 
    {RobXControl::CMD_ENABLE_REGULATOR,  0x37,  2,  0}, 
    {RobXControl::CMD_DISABLE_TIMEOUT,  0x38,  2, 0}, 
    {RobXControl::CMD_ENABLE_TIMEOUT,  0x39, 2, 0}
};
    
class RobXControl::CommImpl
{
public:
    CommImpl(const std::string& device)
    {
        mIO = new boost::asio::io_service();
        mSerial = new boost::asio::serial_port(Service(), device);
        Serial().set_option(boost::asio::serial_port_base::baud_rate(9600));
        Serial().set_option(boost::asio::serial_port_base::character_size(8));
        Serial().set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        Serial().set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::two));
        //TODO: What about handshake? RequestToSendXOnXOff?
        Serial().set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::software));
    }
    ~CommImpl()
    {
        if( Serial().is_open() )
            Serial().close();
    }
    
    boost::asio::io_service&   Service() { return *mIO; }
    boost::asio::serial_port&  Serial() { return *mSerial; }
    
    boost::asio::serial_port*    mSerial;
    boost::asio::io_service*     mIO;
};
    
RobXControl::RobXControl(QObject* parent): QObject(parent)
{
    mMoveTimer = new QTimer(this);
    mTurnTimer = new QTimer(this);
    mTimerFeedback = new QTimer(this);
    
    //TODO: Why these encoders have interval of 1 when feedback interval is 200?
    
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

bool RobXControl::IsOpen()
{
    return mImpl.get() && mImpl->Serial().is_open();
}

void RobXControl::Open(const QString& device)
{
    Close();
    mImpl.reset(new CommImpl(device.toStdString()));
    Init();
}

void RobXControl::Close()
{
    if( IsOpen() )
    {
        mImpl->Serial().close();
    }
}

void RobXControl::Read(unsigned char* buffer, size_t numBytes)
{
    boost::system::error_code ec;
    for(size_t i = 0; i < numBytes; i++)
        boost::asio::read(mImpl->Serial(), boost::asio::buffer(buffer + i, 1), ec);
    
    std::cout << ec.message() << std::endl << std::flush;
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
    
    boost::asio::write(mImpl->Serial(), boost::asio::buffer(mWriteBuffer.data(), info.bytesSend));
}

void RobXControl::Read(size_t numBytes)
{
    if( numBytes == 0 )
        return;
    Read(mReadBuffer, numBytes);
}

void RobXControl::OnMoveTimer()
{
    if( fabsf(mEncoder1) > fabsf(mEncoderGoalForward) || fabsf(mEncoder2) > fabsf(mEncoderGoalForward) )
    {
        Stop();
        mMoveTimer->stop();
        Write(CMD_RESET_ENCODERES);
        
        emit OnCommandDone();
    }
}

void RobXControl::OnTurnTimer()
{
    if( fabsf(mEncoder1) > fabsf(mEncoderGoal) )
    {
        if( fabsf(mEncoder2) > fabsf(mEncoderGoal) )
        {
            Stop();
            mTurnTimer->stop();
            Write(CMD_RESET_ENCODERES);
            
            emit OnCommandDone();
        }
    }
}

void RobXControl::Stop()
{
    Write(CMD_SET_SPEED_1, 128);
    Write(CMD_SET_SPEED_2, 128);
}

void RobXControl::Forward(int cm, float speed_index)
{
    int encoder_amount = (int)(cm * 24.8);
    int speed_amount1 = (int)(128 - speed_index * 10);
    int speed_amount2 = (int)(128 - speed_index * 10);
    
    Write(CMD_SET_SPEED_1, (byte)speed_amount1);
    Write(CMD_SET_SPEED_2, (byte)speed_amount2);
    
    mEncoderGoalForward = encoder_amount;
    mMoveTimer->start();
}

void RobXControl::Backward(int cm, float speed_index)
{
    int encoder_amount = (-1) * (int)(cm * 24.8);
    int speed_amount1 = (int)(128 + speed_index * 10);
    int speed_amount2 = (int)(128 + speed_index * 10);
    
    Write(CMD_SET_SPEED_1, speed_amount1);
    Write(CMD_SET_SPEED_2, speed_amount2);
    
    mEncoderGoalForward = encoder_amount;
    mMoveTimer->start();
}

void RobXControl::TurnLeft(int degree, float speed_index)
{
    int encoder_amount = degree * 11;
    int speed_amount1 = (int)(speed_index * 10 + 128);
    int speed_amount2 = (int)(128 - speed_index * 10);
    
    Write(CMD_SET_SPEED_1, speed_amount1);
    Write(CMD_SET_SPEED_2, speed_amount2);
    
    mEncoderGoal = encoder_amount;
    mTurnTimer->start();
}

void RobXControl::TurnRight(int degree, float speed_index)
{
    int encoder_amount = -1 * degree * 11;
    int speed_amount1 = (int)(128 - speed_index * 10);
    int speed_amount2 = (int)(speed_index * 10 + 128);
    
    Write(CMD_SET_SPEED_1, speed_amount1);
    Write(CMD_SET_SPEED_2, speed_amount2);
    
    mEncoderGoal = encoder_amount;
    mTurnTimer->start();
}

void RobXControl::EnableAutoregulator(bool enable)
{
    if( enable )
        Write(CMD_ENABLE_REGULATOR);
    else
        Write(CMD_DISABLE_REGULATOR);
}

void RobXControl::EnableTimeout(bool enable)
{
    if( enable )
        Write(CMD_ENABLE_TIMEOUT);
    else
        Write(CMD_DISABLE_TIMEOUT);
}

void RobXControl::GetEncoders()
{
    try
    {
        int value = 0;
        unsigned char c;
        std::cout << "\n before write " << std::flush;
        Write(CMD_GET_ENCODER_1);
        std::cout << " after write " << std::flush;
        
        Read((unsigned char*)&value, 4);
//         std::cout << "read \n" << std::flush;
//         Read(&c, 1);
//         std::cout << "1\n" << std::flush;
//         value = c << 24;
//         Read(&c, 1);
//         std::cout << "2\n" << std::flush;
//         value |= c << 16;
//         Read(&c, 1);
//         std::cout << "3\n" << std::flush;
//         value |= c << 8;
//         Read(&c, 1);
//         std::cout << "4\n" << std::flush;
//         value |= c;
        
        mEncoder1 = value;
        
        Write(CMD_GET_ENCODER_2);
        Read((unsigned char*)&value, 4);
//         value = 0;
//         Read(&c, 1);
//         value = c << 24;
//         Read(&c, 1);
//         value |= c << 16;
//         Read(&c, 1);
//         value |= c << 8;
//         Read(&c, 1);
//         value |= c;
        
        mEncoder2 = value;
    }
    catch(std::exception& ex)
    {
        std::cout << "(RobXControl::GetEncoders) Exception occured: " << ex.what() << std::endl << std::flush;
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
    //TODO: Implement
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
    std::cout << "(ONFeedback) before encoderes ... " << std::flush;
    GetEncoders();
    std::cout << " done feedback \n " << std::flush;
}

float RobXControl::GetEncoderValue(int index) const
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


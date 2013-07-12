/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2013  Kourosh <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <roboctrl/SerialPort.h>
#include <boost/bind.hpp>

namespace KSRobot
{
namespace roboctrl
{

namespace io = boost::asio;

SerialPort::SerialPort() : mTimeout(0), mIO(NULL), mPort(NULL), mTimer(NULL)
{
}

SerialPort::SerialPort(const std::string device, int baud_rate, int char_size, const SerialPort::Parity& par, const SerialPort::StopBits& sb, 
                       const SerialPort::FlowControl& fc) : mTimeout(0), mIO(NULL), mPort(NULL), mTimer(NULL)
{
    Open(device, baud_rate, char_size, par, sb, fc);
}

SerialPort::~SerialPort()
{
    Close();
}

bool SerialPort::IsOpen()
{
    //Implementation specific.
    return mIO != NULL && mPort != NULL && mPort->is_open();
}

void SerialPort::Open(const std::string device, int baud_rate, int char_size, const SerialPort::Parity& par, const SerialPort::StopBits& sb, 
                      const SerialPort::FlowControl& fc)
{
    Close();
    
    mIO = new boost::asio::io_service();
    mPort = new boost::asio::serial_port(Service(), device);
    mTimer = new boost::asio::deadline_timer(*mIO);
    
    Port().set_option(BaudRate(baud_rate));
    Port().set_option(boost::asio::serial_port_base::character_size(char_size));
    Port().set_option(par);
    Port().set_option(sb);
    Port().set_option(fc);
}

void SerialPort::Close()
{
    if( IsOpen() )
    {
        Port().close();
        delete mTimer;
        delete mPort;
        delete mIO;
    }
}

common::Duration SerialPort::GetTimeout() const
{
    return mTimeout;
}

void SerialPort::SetTimeout(const common::Duration& dur)
{
    mTimeout = dur;
}

void SerialPort::ReadBlocking(unsigned char* buffer, size_t len, bool byte_by_byte)
{
    assert(IsOpen());
    boost::system::error_code ec;
    //TODO: Check for error
    if( byte_by_byte )
        for(size_t i = 0; i < len; i++)
            io::read(Port(), io::buffer(buffer + i, 1), ec);
    else
        io::read(Port(), io::buffer(buffer, len), ec);
}

void SerialPort::ReadTimout(unsigned char* buffer, size_t len)
{
    assert(IsOpen());
    if( mReadData.size() > 0 )
    {
        std::istream is(&mReadData);
        size_t toRead = std::min(mReadData.size(), len);
        is.read((char*)buffer, toRead);
        buffer += toRead;
        len -= toRead;
    }
    
    if( len == 0 )
        return;
    
    mReadParams = ReadParams(buffer, len);
    SetupRead();
    
    //This code requires a timout, if none is provided set a default (infinite) time    
    if( mTimeout.count() == 0 )
        mTimer->expires_from_now(boost::posix_time::hours(1));
    else
        mTimer->expires_from_now(boost::posix_time::millisec(boost::chrono::duration_cast<boost::chrono::milliseconds>(mTimeout).count()));
    
    mTimer->async_wait(boost::bind(&SerialPort::TimeoutExpired, this, boost::asio::placeholders::error));
    
    mReadResult = RR_IN_PROGRESS;
    mBytesTransfered = 0;
    
    for(;;)
    {
        Service().run_one();
        switch(mReadResult)
        {
            case RR_SUCCESS:
                mTimer->cancel();
                return;
            case RR_TIMEOUT_EXPIRED:
                Port().cancel();
                throw TimeoutException("Serial port timout.");
                return;
            case RR_ERROR:
                mTimer->cancel();
                Port().cancel();
                throw boost::system::system_error(boost::system::error_code(), "Error while reading serial port.");
                return;
            case RR_IN_PROGRESS:
                //nothing
                break;
        }
    }
}

void SerialPort::SetupRead()
{
    boost::asio::async_read(Port(), boost::asio::buffer(mReadParams.Data, mReadParams.Size),
        boost::bind(&SerialPort::OnReadComplete, this, boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
}

void SerialPort::TimeoutExpired(const boost::system::error_code& error)
{
    if( !error && mReadResult == RR_IN_PROGRESS )
        mReadResult = RR_TIMEOUT_EXPIRED;
}

void SerialPort::OnReadComplete(const boost::system::error_code& error, const size_t bytesTransferred)
{
    if( !error )
    {
        mReadResult = RR_SUCCESS;
        mBytesTransfered = bytesTransferred;
        return;
    }
    
#ifdef _WIN32
    if( error.value() == 995 )
        return; //Windows spits out error 995
#else //Linux
    if( error.value() == 125 )
        return; //Linux outputs error 125
#endif
    mReadResult = RR_ERROR;
}

void SerialPort::Write(unsigned char* buffer, size_t bytes)
{
    //TODO: Check for error
    boost::system::error_code ec;
    io::write(Port(), io::buffer(buffer, bytes), ec);
}



};// namespace KSRobot
};// namespace common


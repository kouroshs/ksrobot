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

#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <stdexcept>
#include <boost/asio.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <common/Defenitions.h>

namespace KSRobot
{
namespace roboctrl
{

// Thrown if timeout occures
class TimeoutException : public std::runtime_error
{
public:
    TimeoutException(const std::string& arg) : runtime_error(arg) {;}
};

class SerialPort
{
public:
    typedef boost::asio::serial_port                    SerialPortType;
    typedef boost::asio::io_service                     ServiceType;
    typedef boost::asio::serial_port::baud_rate         BaudRate;
    typedef boost::asio::serial_port::parity            Parity;
    typedef boost::asio::serial_port::stop_bits         StopBits;
    typedef boost::asio::serial_port::flow_control      FlowControl;
    
    SerialPort();
    SerialPort(const std::string device, int baud_rate = 9600, int char_size = 8, 
               const Parity& par = Parity(Parity::none), const StopBits& sb = StopBits(StopBits::two), 
               const FlowControl& fc = FlowControl(FlowControl::software));
    
    ~SerialPort();
    
    ServiceType&                    Service() { assert(mIO != 0); return *mIO; }
    SerialPortType&                 Port() { assert(mPort != 0); return *mPort; }
    
    void                            Close();
    bool                            IsOpen();
    void                            Open(const std::string device, int baud_rate = 9600, int char_size = 8, 
                                         const Parity& par = Parity(Parity::none), const StopBits& sb = StopBits(StopBits::two), 
                                         const FlowControl& fc = FlowControl(FlowControl::software));
    
    common::Duration                GetTimeout() const;
    void                            SetTimeout(const common::Duration& dur);
    
    void                            ReadBlocking(unsigned char* buffer, size_t len, bool byte_by_byte = true);
    void                            ReadTimout(unsigned char* buffer, size_t len);
    
    void                            Write(unsigned char* buffer, size_t bytes);
    
private:
    class ReadParams
    {
    public:
        ReadParams() {;}
        ReadParams(unsigned char* buffer, size_t sz) : Data(buffer), Size(sz) {;}
        
        unsigned char*          Data;
        size_t                  Size;
    };
    
    void                            SetupRead();
    void                            TimeoutExpired(const boost::system::error_code& error);
    void                            OnReadComplete(const boost::system::error_code& error,
                                                   const size_t bytesTransferred);
private:
    ServiceType*                    mIO;
    SerialPortType*                 mPort;
    common::Duration                mTimeout;
    
    enum ReadResult
    {
        RR_SUCCESS,
        RR_IN_PROGRESS,
        RR_TIMEOUT_EXPIRED,
        RR_ERROR
    };
    
    boost::asio::deadline_timer*    mTimer;
    boost::asio::streambuf          mReadData;
    
    ReadResult                      mReadResult;
    size_t                          mBytesTransfered;
    ReadParams                      mReadParams;
};

};// namespace KSRobot
};// namespace common

#endif // SERIALPORT_H

/*
 * <one line to give the library's name and an idea of what it does.>
 * Copyright (C) 2013  Kourosh Sartipi <sartipi@ce.sharid.edu>
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

#include <common/Interface.h>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <stdarg.h>
#include <cstdio>

#define WRITE_TO_BUFFER         va_list args;                                       \
                                va_start(args, format);                             \
                                std::vsprintf(mLogBuffer, format, args);            \
                                va_end(args);


namespace KSRobot
{
namespace common
{

class NullStreamBuffer : public std::streambuf
{
public:
    virtual int overflow(int c)
    {
        return c;
    }
};

static NullStreamBuffer s_nullStreamBuff;
std::ostream Interface::s_NullOutput(&s_nullStreamBuff); // NOTE: Portability issue.

Interface::Interface() : mSleepMillisecs(0.0f), mCycles(0), mLoggerOutputFile("cout"), mLogFile(&std::cout), mLoggingEnabled(true),
    mDebugEnabled(true), mErrorEnabled(true), mMessageEnabled(true)
{
    mContinueExec = true;
}

Interface::~Interface()
{
    if( mContinueExec )
        Stop();
    for(ConnectionList::iterator iter = mConnections.begin(); iter != mConnections.end(); iter++)
        iter->disconnect();
    
    if( mLogFile )
        *mLogFile << std::flush;
    
    if( mLogFile && mLogFile != &std::cout )
        delete mLogFile;
}

void Interface::SpinOnce()
{
    if( !ContinueExecution() || mSleepMillisecs < 0.01 )
        return;
    boost::this_thread::sleep_until(mLastTime + boost::chrono::milliseconds(mSleepMillisecs));
}

void Interface::Start()
{
    mCycles = 0;
    mThread = boost::thread(boost::bind(&Interface::ThreadEntry, this));
}

void Interface::Stop()
{
    mContinueExec = (false);
    if( mThread.joinable() )
        mThread.join();
}

void Interface::ThreadEntry()
{
    while( ContinueExecution() )
    {
        RunSingleCycle();
        SpinOnce();
    }
    mOnFinishSignal();
}

void Interface::WriteRunningTimes(std::ostream& os)
{
    os << "Interface " << mInterfaceName << " times:\n";
    for(size_t i = 0; i < mTimers.size(); i++)
        os << mTimers[i]->ToString() << std::endl;
}

bool Interface::ContinueExecution() const
{
    return mContinueExec;
}

boost::signals2::connection Interface::RegisterOnCycleCompleteReceiver(boost::function<void ()> fn)
{
    return mOnCycleCompleteSignal.connect(fn);
}

boost::signals2::connection Interface::RegisterOnFinishReceiver(boost::function<void ()> fn)
{
    return mOnFinishSignal.connect(fn);
}

void Interface::ReadSettings(ProgramOptions::Ptr po)
{
    ProgramOptions::Ptr logpo = po->StartNode("Logger");
    mLoggingEnabled = logpo->GetBool("Enabled", true);
    mLoggerOutputFile = logpo->GetString("Output", std::string("__interface_name__"));
    
    if( mLoggingEnabled )
    {
        if( mLoggerOutputFile == "__interface_name__" )
            mLogFile = new std::ofstream((GetInterfaceName() + ".txt").c_str());
        else if( mLoggerOutputFile == "cout" )
            mLogFile = &std::cout; //NOTE: This is dangerous for multithreaded programs
        else
            mLogFile = new std::ofstream(mLoggerOutputFile.c_str());
    }
    
    mDebugEnabled = logpo->GetBool("DebugEnabled", true);
    mErrorEnabled = logpo->GetBool("ErrorEnabled", true);
    mMessageEnabled = logpo->GetBool("MessageEnabled", true);
}

void Interface::WriteSettings(ProgramOptions::Ptr po)
{
    //Nothing to do here
    (void)po;
}

void Interface::FinishCycle()
{
    mOnCycleCompleteSignal();
    IncrementCycle();
}

std::ostream& Interface::WriteLog(const char* header, const char* msg)
{
    if( header )
        *mLogFile << header;
    *mLogFile << msg << std::flush;
    return *mLogFile;
}

std::ostream& Interface::Debug(const char* format, ... )
{
    if( !mDebugEnabled || !mLoggingEnabled )
        return s_NullOutput;
    WRITE_TO_BUFFER;
    return WriteLog("DEBUG: ", mLogBuffer);
}

std::ostream& Interface::Error(const char* format, ... )
{
    if( !mErrorEnabled || !mLoggingEnabled )
        return s_NullOutput;
    WRITE_TO_BUFFER;
    return WriteLog("ERROR: ", mLogBuffer);
}

std::ostream& Interface::Message(const char* format, ... )
{
    if( !mMessageEnabled || !mLoggingEnabled )
        return s_NullOutput;
    WRITE_TO_BUFFER;
    return WriteLog("MESSAGE: ", mLogBuffer);
}


} // end namespace common
} // end namespace KSRobot

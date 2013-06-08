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

namespace KSRobot
{
namespace common
{

Interface::Interface(ProgramOptions::Ptr po, const std::string& name) : mPO(po), mSleepMillisecs(0.0f), mName(name)
{
    mContinueExec.store(true);
}

Interface::~Interface()
{
    if( mContinueExec )
        Stop();
}

void Interface::SpinOnce()
{
    if( mSleepMillisecs < 0.01 )
        return;
    TimePoint currTime = Clock::now();
    Duration dur = currTime - mLastTime;
    int milisecs = Milliseconds(dur);
    
    if( milisecs < mSleepMillisecs )
        boost::this_thread::sleep_for(boost::chrono::milliseconds(mSleepMillisecs - milisecs));
    mLastTime = Clock::now();
}

void Interface::Start()
{
    mCycles = 0;
    mThread = boost::thread(boost::bind(&Interface::ThreadEntry, this));
}

void Interface::Stop()
{
    mContinueExec.store(false);
    mThread.join();
}

void Interface::ThreadEntry()
{
    while( ContinueExecution() )
    {
        RunSingleCycle();
        SpinOnce();
    }
}

void Interface::WriteRunningTimes(std::ostream& os)
{
    std::streamsize prec = os.precision(6);
    
    os << "Interface " << mName << " times:\n";
    
    for(size_t i = 0; i < mTimers.size(); i++)
    {
        Timer::Ptr t = mTimers[i];
        
        os << "Timer " << t->Name() << ": Average: " << t->GetAverageTime() << " Variance: " << 
                t->GetTimeVariance() << std::endl;
    }
    
    os.precision(prec);
}


} // end namespace common
} // end namespace KSRobot

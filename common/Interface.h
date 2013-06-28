/*
 * <one line to give the library's name and an idea of what it does.>
 * Copyright (C) 2013  Kourosh <email>
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

#ifndef INTERFACE_H
#define INTERFACE_H

#include <common/Defenitions.h>
#include <common/Timer.h>
#include <common/ProgramOptions.h>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/atomic.hpp>
#include <boost/thread.hpp>
#include <boost/signals2.hpp>
#include <boost/graph/graph_concepts.hpp>

namespace KSRobot
{
namespace common
{

class Interface
{
public:
    class ScopedLock
    {
        Interface* iface;
    public:
        ScopedLock(Interface* i)
        {
            assert(i);
            iface = i;
            iface->LockData();
        }
        
        ~ScopedLock()
        {
            iface->UnlockData();
        }
    };
    
    typedef Interface                           this_type;
    typedef boost::shared_ptr<this_type>        Ptr;
    typedef boost::shared_ptr<const this_type>  ConstPtr;
    
    Interface();
    virtual ~Interface();
    
    inline void                 SetHZ(float hz);
    inline void                 SetPeriod(float t);
    
    inline void                 SetInterfaceName(const std::string& name);
    inline std::string          GetInterfaceName() const;
    
    virtual void                Start();
    virtual void                Stop();
    
    virtual bool                ContinueExecution() const;
    inline void                 LockData();
    inline void                 UnlockData();

    inline int                  GetCycle() const;
    
    inline std::string          GetName() const { return mInterfaceName; }
    // This method is public, to enable running in single threaded context
    virtual bool                RunSingleCycle() = 0;
    virtual void                WriteRunningTimes(std::ostream& os);
    
    virtual void                ReadSettings(ProgramOptions::Ptr po);
    virtual void                WriteSettings(ProgramOptions::Ptr po);
    
    boost::signals2::connection RegisterOnFinishReceiver(boost::function<void()> fn);
    boost::signals2::connection RegisterOnCycleCompleteReceiver(boost::function<void ()> fn);
protected:
    inline void                 RegisterTimer(Timer::Ptr timer);

    virtual void                ThreadEntry();
    virtual void                SpinOnce();
    
    // To be called at the end of RunSingleCycle (in case it's return val is true) to do finilization tasks.
    // Other classes can override it, but must call Interface::FinishCycle
    virtual void                FinishCycle();
    void                        IncrementCycle() { mCycles++; }
protected:
    int                         mSleepMillisecs;
    TimePoint                   mLastTime;
    volatile bool               mContinueExec;
    boost::thread               mThread;
    boost::mutex                mInternalDataGaurd;
    std::vector<Timer::Ptr>     mTimers;
    std::string                 mInterfaceName;
    int                         mCycles;
    
    typedef boost::signals2::signal<void()>  SignalType;
    SignalType                  mOnFinishSignal;
    SignalType                  mOnCycleCompleteSignal;
    
    typedef std::vector<boost::signals2::connection>  ConnectionList;
    ConnectionList              mConnections;
};

inline int Interface::GetCycle() const
{
    return mCycles;
}

inline void Interface::SetHZ(float hz)
{
    mSleepMillisecs = 1000.0f / hz;
}

inline void Interface::SetPeriod(float t)
{
    mSleepMillisecs = t * 1000.0f;
}

inline void Interface::LockData()
{
    mInternalDataGaurd.lock();
}

inline void Interface::UnlockData()
{
    mInternalDataGaurd.unlock();
}

inline void Interface::RegisterTimer(Timer::Ptr timer)
{
    mTimers.push_back(timer);
}

inline std::string Interface::GetInterfaceName() const
{
    return mInterfaceName;
}

void Interface::SetInterfaceName(const std::string& name)
{
    mInterfaceName = name;
}


} // end namespace common
} // end namespace KSRobot

#endif // INTERFACE_H

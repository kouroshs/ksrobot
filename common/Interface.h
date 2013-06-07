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

#include <common/ProgramOptions.h>
#include <common/Defenitions.h>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/atomic.hpp>
#include <boost/thread.hpp>
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
            iface = i;
            iface->LockData();
        }
        
        ~ScopedLock()
        {
            iface->UnlockData();
        }
    };
    
    friend class ScopedLock;
    
    typedef Interface                           this_type;
    typedef boost::shared_ptr<this_type>        Ptr;
    typedef boost::shared_ptr<const this_type>  ConstPtr;
    
    Interface(ProgramOptions::Ptr po, const std::string& name);
    virtual ~Interface();
    
    inline void                 SetHZ(float hz);
    inline void                 SetPeriod(float t);
    
    virtual void                Start();
    virtual void                Stop();
    
    inline virtual bool         ContinueExecution() const;
    inline void                 LockData();
    inline void                 UnlockData();

    inline int                  GetCycle() const;
    
    inline float                GetAverageRunningTime() const;
    inline std::vector<float>   GetRunningTimes() const;
    
    inline std::string          GetName() const { return mName; }
    // This method is public, to enable running in single threaded context
    virtual bool                RunSingleCycle() = 0;
    
private:
    void                        ThreadEntry();
    void                        SpinOnce();
    
protected:
    ProgramOptions::Ptr         mPO;
    int                         mSleepMillisecs;
private:
    TimePoint                   mLastTime;
    boost::atomic<bool>         mContinueExec;
    boost::thread               mThread;
    boost::mutex                mInternalDataGaurd;
    
    std::string                 mName;
    std::vector<float>          mRunningTimes;
    float                       mTotalTime;
    int                         mCycles;
};

inline int Interface::GetCycle() const
{
    return mCycles;
}

inline bool Interface::ContinueExecution() const
{
    return mContinueExec.load();
}

inline float Interface::GetAverageRunningTime() const
{
    return mTotalTime / mCycles;
}

inline std::vector<float> Interface::GetRunningTimes() const
{
    return mRunningTimes;
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

} // end namespace common
} // end namespace KSRobot

#endif // INTERFACE_H

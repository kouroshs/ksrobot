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

#ifndef TIMER_H
#define TIMER_H

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <common/Defenitions.h>

namespace KSRobot
{
namespace common
{

class Timer
{
public:
    typedef boost::shared_ptr<Timer>           Ptr;
    typedef boost::shared_ptr<const Timer>     ConstPtr;
    
    Timer(const std::string& name);
    ~Timer();
    
    void                        Start();
    void                        Stop();
    
    inline double               GetAverageTime() const;
    inline double               GetTimeVariance() const;
    inline size_t               GetNumCalls() const;
    inline double               GetMaxTime() const;
    inline double               GetMinTime() const;
    inline double               GetTotalTime() const;
    
    inline std::string          Name() const;
    
    std::string                 ToString(int precision = 3) const;
private:
    std::string                 mTimerName;
    
    TimePoint                   mStartTime;
    
    typedef boost::accumulators::accumulator_set<double, boost::accumulators::features<boost::accumulators::tag::min, 
            boost::accumulators::tag::max, boost::accumulators::tag::mean, boost::accumulators::tag::moment<2> > > AccType;
    
    AccType                     mAccumulator;
    
    CLASS_DEF_PYEXPORT;
};

inline double Timer::GetAverageTime() const
{
    return boost::accumulators::extract::mean(mAccumulator);
}

inline double Timer::GetTimeVariance() const
{
    return boost::accumulators::moment<2>(mAccumulator);
}

inline std::string Timer::Name() const
{
    return mTimerName;
}

inline double Timer::GetMaxTime() const
{
    return boost::accumulators::extract::max(mAccumulator);
}

inline double Timer::GetMinTime() const
{
    return boost::accumulators::extract::min(mAccumulator);
}

inline size_t Timer::GetNumCalls() const
{
    return boost::accumulators::extract::count(mAccumulator);
}

inline double Timer::GetTotalTime() const
{
    return GetNumCalls() * GetAverageTime();
}


} // end namespace common
} // end namespace KSRobot

#endif // TIMER_H

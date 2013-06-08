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

#include <common/Timer.h>
#include <boost/chrono.hpp>

namespace KSRobot
{
namespace common
{

Timer::Timer(const std::string& name) : mTimerName(name), mCycles(0), mTotalTime(0.0), mTotalSquaredTime(0.0)
{
}

Timer::~Timer()
{
}

void Timer::Start()
{
    mStartTime = Clock::now();
    mCycles++;
}

void Timer::Stop()
{
    TimePoint now = Clock::now();
    Duration d = now - mStartTime;
    
    double time = Seconds(d);
    mTotalTime += time;
    mTotalSquaredTime += time * time;
}

} // end namespace common
} // end namespace KSRobot

/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2013  Kourosh <kourosh.sartipi@gmail.com>
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

#include <common/ThreadContainer.h>

using boost::mutex;
using boost::condition_variable;

namespace KSRobot
{
namespace common
{

ThreadContainer::ThreadContainer()
{
}

ThreadContainer::~ThreadContainer()
{
}

void ThreadContainer::SetConsumerVars(boost::mutex* mtx, boost::condition_variable* cond)
{
    mConsumer.Mutex = mtx;
    mConsumer.CondVar = cond;
}

void ThreadContainer::SetProducerVars(boost::mutex* mtx, boost::condition_variable* cond)
{
    mProducer.Mutex = mtx;
    mProducer.CondVar = cond;
}

void ThreadContainer::Run()
{
    mContinueRunning = true;
    
    while( mContinueRunning )
    {
        if( mProducer.IsUsable() )
        {
            // then this is a producer
            Produce();
        }
        if( mConsumer.IsUsable() )
        {
            // then this is a consumer
            Consume();
        }
    }
}

void ThreadContainer::SendStop()
{
    mContinueRunning = false;
}

void ThreadContainer::Consume()
{
    //TODO: how can I prevent spurious wakups?
    
    
    
}

void ThreadContainer::Produce()
{

}



} // end namespace utils
} // end namespace KSRobot

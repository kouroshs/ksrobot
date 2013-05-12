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

#ifndef THREADCONTAINER_H
#define THREADCONTAINER_H

#include <boost/thread.hpp>
#include <boost/signals2.hpp>

//extern gtsam::Point2 P;
namespace KSRobot
{
namespace utils
{
        
class ThreadContainer
{
public:
    ThreadContainer();
    ~ThreadContainer();

    void                                        SetConsumerVars(boost::mutex* mtx, boost::condition_variable* cond);
    void                                        SetProducerVars(boost::mutex* mtx, boost::condition_variable* cond);
    
    
    void                                        Run();
    
    void                                        SendStop();
    
private:
    void                                        Produce();
    void                                        Consume();
private:
    class UserInfo
    {
    public:
        UserInfo() : Mutex(NULL), CondVar(NULL) {;}
        
        bool                                    IsUsable() { return Mutex != NULL && CondVar != NULL; }
        
        boost::mutex*                           Mutex;
        boost::condition_variable*              CondVar;
    };

    UserInfo                                    mConsumer;
    UserInfo                                    mProducer;
    
    volatile bool                               mContinueRunning;
};

} // end namespace utils
} // end namespace KSRobot


#endif // THREADCONTAINER_H

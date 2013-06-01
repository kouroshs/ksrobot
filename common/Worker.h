/*
 * <one line to give the library's name and an idea of what it does.>
 * Copyright (C) 2013  Kourosh <kourosh.sartipi@gmail.com>
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

#ifndef KSROBOT_UTILS_WORKER_H
#define KSROBOT_UTILS_WORKER_H

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/chrono/include.hpp>


namespace KSRobot
{
namespace common
{

class IConsumer
{
public:
    typedef boost::shared_ptr<IConsumer> Ptr;
    typedef boost::shared_ptr<const IConsumer> ConstPtr;
    
    virtual ~IConsumer() {;}
    
    virtual void                ConsumeFast() {;}
    virtual void                ConsumeComplete() {;}
    virtual void                OnConsumeStop() {;}
};

class IProducer
{
public:
    typedef boost::shared_ptr<IProducer> Ptr;
    typedef boost::shared_ptr<const IProducer> ConstPtr;
    
    virtual ~IProducer() {;}
    
    virtual void                Produce() {;}
    virtual void                OnProduceFinish() {;}
};

class IConsumerProducer : public IConsumer, public IProducer
{
public:
    typedef boost::shared_ptr<IConsumerProducer> Ptr;
    typedef boost::shared_ptr<const IConsumerProducer> ConstPtr;
};

class Worker
{
public:
    typedef boost::shared_ptr<Worker> Ptr;
    typedef boost::shared_ptr<const Worker> ConstPtr;
    typedef boost::function<void ()>  CallerType;
    typedef boost::chrono::high_resolution_clock::time_point TimePoint;
    
    Worker();
    //Worker(bool isProducer);
    ~Worker();
    
    static CallerType                           DefaultFunction();
    
    static Worker::Ptr                          New();
    
    void                                        SetConsumer(IConsumer::Ptr consumer);
    void                                        SetProducer(IProducer::Ptr producer);
    void                                        SetConsumerProducer(IConsumerProducer::Ptr conprod);
    
    size_t                                      ListenTo(Worker::Ptr wrk);
    void                                        DetachListener(size_t id);
    void                                        DetachAllListeners();
    
    bool                                        IsConsumer() const;
    bool                                        IsProducer() const;
    
    void                                        StartWorker(bool seperateThread = true);
    void                                        StopWorker(bool wait = true);
    
    void                                        SetWorkerName(const std::string& name) { mWorkerName = name; }
    std::string                                 GetWorkerName() const { return mWorkerName; }
    
    void                                        DoProduce();
    bool                                        DoConsume();
    
    static IConsumer::Ptr                       CreateConsumer(CallerType fast, CallerType complete = 0, 
                                                               CallerType onfinish = 0);
    static IConsumer::Ptr                       CreateEmptyConsumer();
    
    static IProducer::Ptr                       CreateProducer(CallerType produce, CallerType onfinish = 0);
    static IProducer::Ptr                       CreateEmptyProducer();
    static IConsumerProducer::Ptr               CreateConsumerProducer(CallerType fast_consumer,
                                                                       CallerType complete_consumer,
                                                                       CallerType onfinish_consumer,
                                                                       CallerType produce,
                                                                       CallerType onfinish_producer);
    static IConsumerProducer::Ptr               CreateEmptyConsumerProducer();
private:
    void                                        ThreadEntry();
    void                                        DoFastConsume();
    void                                        CallProduceProc();
private:
    class WaitCond
    {
    public:
        typedef boost::shared_ptr<WaitCond> Ptr;
        WaitCond() : Stop(false) {;}
        
        boost::mutex                            Locker;
        boost::condition_variable               CondVar;
        bool                                    Stop;
        TimePoint                               LastSignalTime;
    };
    
    WaitCond::Ptr                               mConsumerInfo;
    IConsumer::Ptr                              mConsumerInterface;
    TimePoint                                   mLastConsumedSignalTime;
    
    WaitCond::Ptr                               mProducerInfo;
    IProducer::Ptr                              mProducerInterface;
    
    typedef std::map<size_t, Worker::Ptr>       ListenerMap;
    ListenerMap                                 mListenerMap;
    
    boost::thread                               mExecThread;
    bool                                        mContinueExec;
    
    bool                                        mConsumerFinishCalled;
    bool                                        mProducerFinishCalled;
    
    std::string                                 mWorkerName;
    
    static size_t                               m_sLastID;
    static boost::mutex                         m_sLock;
    static size_t                               GenerateID();
};

}
}

#endif // KSROBOT_UTILS_WORKER_H

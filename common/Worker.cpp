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

#include <common/Worker.h>

namespace KSRobot
{
namespace common
{

    #define SAFE_CALL(x)  if( (x) ) x();
    class DefConsumer : public IConsumer
    {
    public:
        typedef boost::shared_ptr<DefConsumer> Ptr;
        
        DefConsumer(Worker::CallerType fast, Worker::CallerType comp, Worker::CallerType fin) : 
        mFast(fast), mComp(comp), mFinCon(fin) { ; }
        ~DefConsumer() {;}
        
        virtual void ConsumeFast() { SAFE_CALL(mFast); }
        virtual void ConsumeComplete() { SAFE_CALL(mComp); }
        virtual void OnConsumeStop() { SAFE_CALL(mFinCon); }
        
    private:
        Worker::CallerType  mFast;
        Worker::CallerType  mComp;
        Worker::CallerType  mFinCon;
    };
    
    class DefProducer : public IProducer
    {
    public:
        typedef boost::shared_ptr<DefProducer> Ptr;
        
        DefProducer(Worker::CallerType prd, Worker::CallerType comp) :
        mProd(prd), mFinProd(comp) {;}
        
        virtual void Produce() { SAFE_CALL(mProd); }
        virtual void OnProduceFinish() { SAFE_CALL(mFinProd); }
    private:
        Worker::CallerType  mProd;
        Worker::CallerType  mFinProd;
    };
    
    class DefConsumerProducer : public IConsumerProducer
    {
    public:
        typedef boost::shared_ptr<DefConsumerProducer> Ptr;
        
        DefConsumerProducer(Worker::CallerType fast, Worker::CallerType comp, 
                            Worker::CallerType fin_con, Worker::CallerType prd,
                            Worker::CallerType fin_prd) : mProd(prd), mFinProd(fin_prd),
                            mFast(fast), mComp(comp), 
                            mFinCon(fin_con) { ; }
                            
                            virtual void Produce() { SAFE_CALL(mProd); }
                            virtual void OnProduceFinish() { SAFE_CALL(mFinProd); }
                            virtual void ConsumeFast() { SAFE_CALL(mFast); }
                            virtual void ConsumeComplete() { SAFE_CALL(mComp); }
                            virtual void OnConsumeStop() { SAFE_CALL(mFinCon); }
                            
    private:
        Worker::CallerType  mProd;
        Worker::CallerType  mFinProd;
        Worker::CallerType  mFast;
        Worker::CallerType  mComp;
        Worker::CallerType  mFinCon;
    };
    
    size_t Worker::m_sLastID;
    boost::mutex Worker::m_sLock;
    
    Worker::Worker() : mContinueExec(false), mConsumerFinishCalled(false), mProducerFinishCalled(false)
    {
        
    }
    
    Worker::~Worker()
    {
        StopWorker(true);
    }
    
    Worker::Ptr Worker::New()
    {
        return Worker::Ptr(new Worker);
    }
    
    void dummyfn()
    {
    }
    
    Worker::CallerType Worker::DefaultFunction()
    {
        return boost::bind(dummyfn);
    }
    
    
    size_t Worker::GenerateID()
    {
        boost::mutex::scoped_lock lock(m_sLock);
        size_t retVal = ++m_sLastID;
        return retVal;
    }
    
    IConsumer::Ptr Worker::CreateConsumer(Worker::CallerType fast, Worker::CallerType complete, Worker::CallerType onfinish)
    {
        DefConsumer::Ptr ptr(new DefConsumer(fast, complete, onfinish));
        return boost::dynamic_pointer_cast<IConsumer>(ptr);
    }
    
    IConsumer::Ptr Worker::CreateEmptyConsumer()
    {
        return CreateConsumer(DefaultFunction(), DefaultFunction(), DefaultFunction());
    }
    
    IProducer::Ptr Worker::CreateProducer(Worker::CallerType produce, Worker::CallerType onfinish)
    {
        DefProducer::Ptr ptr(new DefProducer(produce, onfinish));
        return boost::dynamic_pointer_cast<IProducer>(ptr);
    }
    
    IProducer::Ptr Worker::CreateEmptyProducer()
    {
        return CreateProducer(DefaultFunction(), DefaultFunction());
    }
    
    IConsumerProducer::Ptr Worker::CreateConsumerProducer(Worker::CallerType fast_consumer, 
                                                          Worker::CallerType complete_consumer, 
                                                          Worker::CallerType onfinish_consumer, 
                                                          Worker::CallerType produce, 
                                                          Worker::CallerType onfinish_producer)
    {
        DefConsumerProducer::Ptr ptr(new DefConsumerProducer(fast_consumer,
                                                             complete_consumer,
                                                             onfinish_consumer,
                                                             produce,
                                                             onfinish_producer));
        IConsumerProducer::Ptr ret = boost::dynamic_pointer_cast<IConsumerProducer>(ptr);
        return ret;
    }
    
    IConsumerProducer::Ptr Worker::CreateEmptyConsumerProducer()
    {
        return CreateConsumerProducer(DefaultFunction(), DefaultFunction(), DefaultFunction(),
                                      DefaultFunction(), DefaultFunction());
    }
    
    
    void Worker::SetConsumer(IConsumer::Ptr consumer)
    {
        mConsumerInterface = consumer;
    }
    
    void Worker::SetProducer(IProducer::Ptr producer)
    {
        mProducerInterface = producer;
        mProducerInfo = WaitCond::Ptr(new WaitCond);
    }
    
    void Worker::SetConsumerProducer(IConsumerProducer::Ptr conprod)
    {
        SetConsumer(conprod);
        SetProducer(conprod);
    }
    
    bool Worker::IsConsumer() const
    {
        return mConsumerInfo.get() && mConsumerInterface.get();
    }
    
    bool Worker::IsProducer() const
    {
        return mProducerInfo.get() && mProducerInterface.get();
    }
    
    size_t Worker::ListenTo(Worker::Ptr wrk)
    {
        //TODO: Check this with SetConsumerProducer
        size_t id = GenerateID();
        mConsumerInfo = wrk->mProducerInfo;
        mListenerMap[id] = wrk;
        return id;
    }
    
    void Worker::DetachListener(size_t id)
    {
        /*
         *    ListenerMap::iterator iter = mListenerMap.find(id);
         *    if( iter != mListenerMap.end() )
         *    {
         *        
         *        mListenerMap.erase(iter);
         }*/
        //NOTE: Dont really think it is necessary for my thesis.
        boost::throw_exception(std::runtime_error("(Worker::DetachListener) NOT IMPLEMENTED."));
    }
    
    void Worker::DetachAllListeners()
    {
        boost::throw_exception(std::runtime_error("(Worker::DetachAllListeners) NOT IMPLEMENTED."));
    }
    
    void Worker::StartWorker(bool seperateThread)
    {
        mContinueExec = true;
        if( seperateThread )
        {
            mExecThread = boost::thread(boost::bind(&Worker::ThreadEntry, this));
        }
        else
        {
            // This means that the worker will be called from outside.
        }
    }
    
    void Worker::StopWorker(bool wait)
    {
        if( mContinueExec == false ) // Worker has not been started
            return;
        
        mContinueExec = false;
        if( IsConsumer() )
        {
            //Notify the consume to wake up
            mConsumerInfo->CondVar.notify_all();
        }
        
        if( mExecThread.joinable() )
            mExecThread.join();
    }
    
    void Worker::ThreadEntry()
    {
        //bool consumerFinishCalled = false, producerFinishCalled = false;
        while( mContinueExec )
        {
            if( DoConsume() == false ) // spurious wakup
                continue;
            DoProduce();
        }
        
        if( mConsumerFinishCalled == false && IsConsumer() )
            mConsumerInterface->OnConsumeStop();
        if( mProducerFinishCalled == false && IsProducer() )
            mProducerInterface->OnProduceFinish();
    }
    
    bool Worker::DoConsume()
    {
        if( IsConsumer() )
        {
            //TODO: always check if the consumer is exiting after wait.
            //First check for spurious wakeups
            if( (mConsumerInfo->LastSignalTime < mLastConsumedSignalTime) && 
                mLastConsumedSignalTime != TimePoint() )
                return false;
            
            DoFastConsume();
            if( mContinueExec )
            {
                mConsumerInterface->ConsumeComplete();
                mConsumerFinishCalled = true;
                mConsumerInterface->OnConsumeStop();
            }
            return true;
        }
        return true;
    }
    
    void Worker::DoProduce()
    {
        if( IsProducer() && mContinueExec == true )
        {
            CallProduceProc();
            //NOTE: This sleep is to give the consumers time to aquire lock.
            //NOTE: Don't sleep if this is a consumer thread, since it has to wait
            //      on a producer anyway!
            if( !IsConsumer() )
            {
                //boost::this_thread::sleep(boost::chrono::milliseconds(1));
                boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
                //boost::this_thread::yield();// after many tests I don't think it has any effects
            }
        }
    }
    
    void Worker::DoFastConsume()
    {
        boost::mutex::scoped_lock lock(mConsumerInfo->Locker);
        mConsumerInfo->CondVar.wait(lock);
        mLastConsumedSignalTime = mConsumerInfo->LastSignalTime;
        if( mContinueExec )
            mConsumerInterface->ConsumeFast();
    }
    
    void Worker::CallProduceProc()
    {
        boost::mutex::scoped_lock lock(mProducerInfo->Locker);
        mProducerInterface->Produce();
        mProducerInfo->LastSignalTime = boost::chrono::high_resolution_clock::now();
        mProducerInfo->CondVar.notify_all();
    }
    

}
}
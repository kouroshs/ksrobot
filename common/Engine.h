#ifndef ENGINE_H
#define ENGINE_H

#include <common/ProgramOptions.h>
#include <common/ExecCtrlData.h>
#include <common/Worker.h>
#include <common/KinectInterface.h>

#include <common/FovisInterface.h>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>

#include <vector>


namespace KSRobot
{
namespace common
{

namespace detail
{
    class KinectThread;
    class FovisThread;
    class SAMThread;
    class OctoMapThread;
    class OMPLThread;
    class CommThread;
};
    
class Engine
{
public:
    Engine(const ProgramOptions::Ptr po);
    ~Engine();
    
    void                                Start(const ExecCtrlData& ed);
    void                                Stop();

//     boost::signals2::connection         RegisterKinectRGBDReceiver();
//     boost::signals2::connection         RegisterKinectPointCloudReceiver();
//     boost::signals2::connection         RegisterFovisReceiver();
//     boost::signals2::connection         RegisterSAMReceiver();
//     boost::signals2::connection         RegisterOctoMapReceiver();
//     boost::signals2::connection         RegisterOMPLReceiver();
    
    const ExecCtrlData&                 GetExecControlData() const;
    
private:
    // Setup functions
    void                                SetupKinect();
    void                                SetupFovis();
    void                                SetupOctoMap();
    void                                SetupOMPL();
    void                                SetupSAM();
    
    
    void                                KinectRGBD(KinectRgbImage::Ptr rgb, KinectFloatDepthImage::Ptr depth);
    void                                KinectPC(const KinectPointCloud::ConstPtr& p);
private:
    typedef boost::shared_ptr<fovis::VisualOdometry>  VisualOdometryPtr;

    ProgramOptions::Ptr                 mPO;
    KinectInterface::Ptr                mKinectInterface;
    VisualOdometryPtr                   mVO;
    
    ExecCtrlData                        mExecCtrl;
    
    Worker::Ptr                         mKinectWorker;
    Worker::Ptr                         mFovisWorker;
    Worker::Ptr                         mSAMWorker;
    Worker::Ptr                         mOctoMapWorker;
    Worker::Ptr                         mOMPLWorker;
    Worker::Ptr                         mCommWorker;
    
    friend class detail::KinectThread;
    friend class detail::FovisThread;
    friend class detail::SAMThread;
    friend class detail::OctoMapThread;
    friend class detail::OMPLThread;
    friend class detail::CommThread;
    
    boost::shared_ptr<detail::KinectThread>           mKinectThread;
    boost::shared_ptr<detail::FovisThread>            mFovisThread;
    boost::shared_ptr<detail::SAMThread>              mSAMThread;
    boost::shared_ptr<detail::OctoMapThread>          mOctoMapThread;
    boost::shared_ptr<detail::OMPLThread>             mOMPLThread;
    boost::shared_ptr<detail::CommThread>             mCommThread;
};

namespace detail
{
    class KinectThread : public IProducer
    {
    public:
        KinectThread(Engine* eng) : mEngine(eng) {;}
        
        virtual void                Produce();
        virtual void                OnProduceFinish();
        
        friend class Engine;
        
        Engine* mEngine;
    };
    
    class FovisThread : public IConsumerProducer
    {
    public:
        FovisThread(Engine* eng) : mEngine(eng) {;}
        
        virtual void                ConsumeFast();
        virtual void                ConsumeComplete();
        virtual void                OnConsumeStop();
        virtual void                Produce();
        virtual void                OnProduceFinish();

        friend class Engine;
        Engine* mEngine;
    };
    
    class SAMThread : public IConsumerProducer
    {
    public:
        SAMThread(Engine* eng) : mEngine(eng) {;}
        
        virtual void                ConsumeFast();
        virtual void                ConsumeComplete();
        virtual void                OnConsumeStop();
        virtual void                Produce();
        virtual void                OnProduceFinish();

        friend class Engine;
        Engine* mEngine;
    };
    
    class OctoMapThread : public IConsumerProducer
    {
    public:
        OctoMapThread(Engine* eng) : mEngine(eng) {;}
        
        virtual void                ConsumeFast();
        virtual void                ConsumeComplete();
        virtual void                OnConsumeStop();
        virtual void                Produce();
        virtual void                OnProduceFinish();

        friend class Engine;
        Engine* mEngine;
    };
    
    class OMPLThread : public IConsumerProducer
    {
    public:
        OMPLThread(Engine* eng) : mEngine(eng) {;}
        virtual void                ConsumeFast();
        virtual void                ConsumeComplete();
        virtual void                OnConsumeStop();
        virtual void                Produce();
        virtual void                OnProduceFinish();

        friend class Engine;
        Engine* mEngine;
    };
    
    class CommThread : public IConsumerProducer
    {
    public:
        CommThread(Engine* eng) : mEngine(eng) {;}
        
        virtual void                ConsumeFast();
        virtual void                ConsumeComplete();
        virtual void                OnConsumeStop();
        virtual void                Produce();
        virtual void                OnProduceFinish();

        friend class Engine;
        Engine* mEngine;
    };
};

} // end namespace utils
} // end namespace KSRobot

#endif // ENGINE_H

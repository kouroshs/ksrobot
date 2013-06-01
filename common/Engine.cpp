#include <common/Engine.h>

#include <common/kinect/KinectDatasetReader.h>
#include <common/kinect/KinectDeviceReader.h>

#include <boost/bind.hpp>
#include <boost/filesystem.hpp>

namespace KSRobot
{
namespace common
{

namespace fs = boost::filesystem;

Engine::Engine(const ProgramOptions::Ptr po) : mPO(po)
{
    mKinectThread.reset(new detail::KinectThread(this));
    mFovisThread.reset(new detail::FovisThread(this));
    mSAMThread.reset(new detail::SAMThread(this));
    mOctoMapThread.reset(new detail::OctoMapThread(this));
    mOMPLThread.reset(new detail::OMPLThread(this));
    mCommThread.reset(new detail::CommThread(this));
    
    mKinectWorker = Worker::New();
    mFovisWorker = Worker::New();
    mSAMWorker = Worker::New();
    mOctoMapWorker = Worker::New();
    mOMPLWorker = Worker::New();
    mCommWorker = Worker::New();
}

Engine::~Engine()
{
}

void Engine::Start(const ExecCtrlData& ed)
{
    mExecCtrl = ed;
    // now see what systems should we run
    mExecCtrl.CheckConsistancy();
    
    SetupKinect();
    SetupFovis();
    SetupSAM();
    SetupOctoMap();
    SetupOMPL();
    
    mKinectInterface->Start();
}

void Engine::Stop()
{
    mKinectInterface->Stop();
    
    mKinectWorker->StopWorker();
    mFovisWorker->StopWorker();
    mOctoMapWorker->StopWorker();
}

void Engine::SetupKinect()
{
    std::string kinectStr;
    if( mExecCtrl.Kinect.GetFromDevice )
    {
        mKinectInterface.reset(new KinectDeviceReader(mPO->StartNode("KinectDeviceReader")));
        kinectStr = mExecCtrl.Kinect.SourceDevice;
    }
    else
    {
        mKinectInterface.reset(new KinectDatasetReader(mPO->StartNode("KinectDatasetReader")));
        kinectStr = mExecCtrl.Kinect.SourceDir;
    }
    mKinectInterface->Initialize(kinectStr);
    mKinectWorker->SetProducer(mKinectThread);
    mKinectWorker->StartWorker(false);
}

void Engine::SetupFovis()
{
    if( !mExecCtrl.Fovis.Enable )
        return;
    
    mFovisWorker->ListenTo(mKinectWorker);
    mFovisWorker->SetConsumerProducer(mFovisThread);
    mFovisWorker->StartWorker();
}

void Engine::SetupSAM()
{
    if( !mExecCtrl.iSAM.Enable )
        return;
    
    mSAMWorker->ListenTo(mFovisWorker);
    mSAMWorker->SetConsumerProducer(mSAMThread);
}

void Engine::SetupOctoMap()
{
    if( !mExecCtrl.OctoMap.Enable )
        return;
    
    mOctoMapWorker->ListenTo(mFovisWorker);
    mOctoMapWorker->SetConsumerProducer(mOctoMapThread);
}

void Engine::SetupOMPL()
{
    if( !mExecCtrl.OMPL.Enable )
        return;
    //TODO: What to do really???
    mOMPLWorker->ListenTo(mOctoMapWorker);
    mOMPLWorker->SetConsumerProducer(mOMPLThread);
}

const ExecCtrlData& Engine::GetExecControlData() const
{
    return mExecCtrl;
}


void Engine::KinectPC(const KinectPointCloud::ConstPtr& p)
{
//TODO: Complete this
}

void Engine::KinectRGBD(KinectRgbImage::Ptr rgb, KinectFloatDepthImage::Ptr depth)
{

}


namespace detail
{

//Kinect Thread

void KinectThread::Produce()
{
    KSRobot::common::IProducer::Produce();
}

void KinectThread::OnProduceFinish()
{
    KSRobot::common::IProducer::OnProduceFinish();
}

// Fovis Thread

void FovisThread::ConsumeFast()
{
    KSRobot::common::IConsumer::ConsumeFast();
}

void FovisThread::ConsumeComplete()
{
    KSRobot::common::IConsumer::ConsumeComplete();
}

void FovisThread::OnConsumeStop()
{
    KSRobot::common::IConsumer::OnConsumeStop();
}

void FovisThread::Produce()
{
    KSRobot::common::IProducer::Produce();
}

void FovisThread::OnProduceFinish()
{
    KSRobot::common::IProducer::OnProduceFinish();
}

// iSAM Thread

void SAMThread::ConsumeFast()
{
    KSRobot::common::IConsumer::ConsumeFast();
}

void SAMThread::ConsumeComplete()
{
    KSRobot::common::IConsumer::ConsumeComplete();
}

void SAMThread::OnConsumeStop()
{
    KSRobot::common::IConsumer::OnConsumeStop();
}

void SAMThread::Produce()
{
    KSRobot::common::IProducer::Produce();
}

void SAMThread::OnProduceFinish()
{
    KSRobot::common::IProducer::OnProduceFinish();
}

// OctoMap

void OctoMapThread::ConsumeFast()
{
    KSRobot::common::IConsumer::ConsumeFast();
}

void OctoMapThread::ConsumeComplete()
{
    KSRobot::common::IConsumer::ConsumeComplete();
}

void OctoMapThread::OnConsumeStop()
{
    KSRobot::common::IConsumer::OnConsumeStop();
}

void OctoMapThread::Produce()
{
    KSRobot::common::IProducer::Produce();
}

void OctoMapThread::OnProduceFinish()
{
    KSRobot::common::IProducer::OnProduceFinish();
}

// OMPL Thread

void OMPLThread::ConsumeFast()
{
    KSRobot::common::IConsumer::ConsumeFast();
}

void OMPLThread::ConsumeComplete()
{
    KSRobot::common::IConsumer::ConsumeComplete();
}

void OMPLThread::OnConsumeStop()
{
    KSRobot::common::IConsumer::OnConsumeStop();
}

void OMPLThread::Produce()
{
    KSRobot::common::IProducer::Produce();
}

void OMPLThread::OnProduceFinish()
{
    KSRobot::common::IProducer::OnProduceFinish();
}

// Comm thread

void CommThread::ConsumeFast()
{
    KSRobot::common::IConsumer::ConsumeFast();
}

void CommThread::ConsumeComplete()
{
    KSRobot::common::IConsumer::ConsumeComplete();
}

void CommThread::OnConsumeStop()
{
    KSRobot::common::IConsumer::OnConsumeStop();
}

void CommThread::Produce()
{
    KSRobot::common::IProducer::Produce();
}

void CommThread::OnProduceFinish()
{
    KSRobot::common::IProducer::OnProduceFinish();
}

} // end of namespace detail


} // end namespace utils
} // end namespace KSRobot

#include <interfaces/KinectDeviceReader.h>
#include <iostream>

namespace KSRobot
{
namespace interfaces
{

KinectDeviceReader::KinectDeviceReader(common::ProgramOptions::Ptr po) : KinectInterface(po), mCurrRgb(new common::KinectRgbImage()),
                                                                mCurrDepthFloat(new common::KinectFloatDepthImage())
{
}

KinectDeviceReader::~KinectDeviceReader()
{
}

void KinectDeviceReader::Initialize(const std::string& device)
{
    mGrabber.reset(new pcl::OpenNIGrabber(device));
}

bool KinectDeviceReader::IsRunning()
{
    return mGrabber->isRunning();
}

boost::signals2::connection KinectDeviceReader::RegisterPointCloudCallback(boost::function<KinectInterface::PointCloudReceiverFn> fn)
{
    return mGrabber->registerCallback(fn);
}

boost::signals2::connection KinectDeviceReader::RegisterRGBDFloatCallback(boost::function<KinectInterface::RGBDReceiverFn> fn)
{
    return mRGBDFloatReceivers.connect(fn);
}

boost::signals2::connection KinectDeviceReader::RegisterRGBDRawCallback(boost::function< KinectInterface::RGBDRawReceiverFn > fn)
{
    return mRGBDRawReceivers.connect(fn);
}


void KinectDeviceReader::Start()
{
    mGrabber->start();
}

void KinectDeviceReader::Stop()
{
    mGrabber->stop();
}

void KinectDeviceReader::RGBDCallback(const boost::shared_ptr<openni_wrapper::Image>& rgb, 
                                      const boost::shared_ptr<openni_wrapper::DepthImage>& depth, 
                                      float /*invFocalLength*/)
{
    // Only do this if there is registered function
    if( mRGBDFloatReceivers.empty() && mRGBDRawReceivers.empty() )
        return;
    
    mCurrRgb->Create(rgb->getWidth(), rgb->getHeight());
    rgb->fillRGB(rgb->getHeight(), rgb->getWidth(), mCurrRgb->GetArray().data());

    if( !mRGBDFloatReceivers.empty() )
    {
        mCurrDepthFloat->Create(depth->getWidth(), depth->getHeight());
        depth->fillDepthImage(depth->getHeight(), depth->getWidth(), mCurrDepthFloat->GetArray().data());
    }
    
    if( !mRGBDRawReceivers.empty() )
    {
        mCurrDepthRaw->Create(depth->getWidth(), depth->getHeight());
        depth->fillDepthImageRaw(depth->getHeight(), depth->getWidth(), mCurrDepthRaw->GetArray().data());
    }
    
    mRGBDFloatReceivers(mCurrRgb, mCurrDepthFloat);
    mRGBDRawReceivers(mCurrRgb, mCurrDepthRaw);
}



} // end namespace utils
} // end namespace KSRobot

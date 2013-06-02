#ifndef KINECTDEVICEREADER_H
#define KINECTDEVICEREADER_H

#include <common/KinectInterface.h>
#include <pcl/io/openni_grabber.h>

namespace KSRobot
{
namespace interfaces
{

class KinectDeviceReader : public common::KinectInterface
{
public:
    typedef boost::shared_ptr<KinectDeviceReader>               Ptr;
    typedef boost::shared_ptr<const KinectDeviceReader>         ConstPtr;
    
    KinectDeviceReader(common::ProgramOptions::Ptr po);
    virtual ~KinectDeviceReader();
    
    virtual void                        Initialize(const std::string& device);
    virtual void                        Start();
    virtual void                        Stop();
    virtual bool                        IsRunning();
    virtual boost::signals2::connection RegisterRGBDRawCallback(boost::function<RGBDRawReceiverFn> fn);
    virtual boost::signals2::connection RegisterRGBDFloatCallback(boost::function<RGBDReceiverFn> fn);
    virtual boost::signals2::connection RegisterPointCloudCallback(boost::function<PointCloudReceiverFn> fn);
    
private:
    void                                RGBDCallback(const boost::shared_ptr<openni_wrapper::Image>&, 
                                                     const boost::shared_ptr<openni_wrapper::DepthImage>&, 
                                                     float constant);
private:
    boost::shared_ptr<pcl::OpenNIGrabber>       mGrabber;
    common::KinectRgbImage::Ptr                 mCurrRgb;
    common::KinectFloatDepthImage::Ptr          mCurrDepthFloat;
    common::KinectRawDepthImage::Ptr            mCurrDepthRaw;
    
    boost::signals2::signal<RGBDReceiverFn>     mRGBDFloatReceivers;
    boost::signals2::signal<RGBDRawReceiverFn>  mRGBDRawReceivers;
    
};

} // end namespace utils
} // end namespace KSRobot

#endif // KINECTDEVICEREADER_H

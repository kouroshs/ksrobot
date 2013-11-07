#include <interfaces/KinectDeviceReader.h>
#include <iostream>
#include <pcl/io/openni_grabber.h>
#include <limits>

namespace KSRobot
{
namespace interfaces
{

class KinectDeviceReader::GrabberHelper : public pcl::OpenNIGrabber
{
public:
    GrabberHelper(const std::string& device_id = "", const Mode& depth_mode = OpenNI_Default_Mode, 
                  const Mode& image_mode = OpenNI_Default_Mode) : pcl::OpenNIGrabber(device_id, depth_mode, image_mode) {;}
    
    virtual ~GrabberHelper() throw() {;}
    
    common::KinectPointCloud::Ptr GeneratePC(const boost::shared_ptr<openni_wrapper::Image> &image,
                                             const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image) const
    {
        return convertToXYZRGBPointCloud<pcl::PointXYZRGBA>(image, depth_image);
    }
    
};

KinectDeviceReader::KinectDeviceReader() : common::KinectInterface()
{
    SetInterfaceName("KinectDeviceReader");
    
    memset(&mParams, 0, sizeof(mParams));
    mParams.Width = 640;
    mParams.Height = 480;
    mParams.FocalX = 528.49404721f;
    mParams.FocalY = 528.49404721f;
    mParams.CenterX = mParams.Width / 2 - 0.5f;
    mParams.CenterY = mParams.Height / 2 - 0.5f;
}

KinectDeviceReader::~KinectDeviceReader()
{
}

void KinectDeviceReader::Initialize(const std::string& device)
{
    mGrabber.reset(new GrabberHelper(device));
    
    boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&, 
                          const boost::shared_ptr<openni_wrapper::DepthImage>& , 
                          float)> fn;
    
    assert(mGrabber.get());
    
    fn = boost::bind(&KinectDeviceReader::RGBDCallback, this, _1, _2, _3);
    boost::signals2::connection c =  mGrabber->registerCallback(fn);
    assert(c.connected());
    mConnections.push_back(c);
}

bool KinectDeviceReader::ContinueExecution() const
{
    assert(mGrabber.get());
    return mGrabber->isRunning();
}

void KinectDeviceReader::Start()
{
    assert(mGrabber.get());
    mGrabber->start();
}

void KinectDeviceReader::Stop()
{
    assert(mGrabber.get());
    mGrabber->stop();
}

void KinectDeviceReader::RGBDCallback(const boost::shared_ptr<openni_wrapper::Image>& rgb, 
                                      const boost::shared_ptr<openni_wrapper::DepthImage>& depth, 
                                      float /*invFocalLength*/)
{
    common::Interface::ScopedLock lock(this);
    assert(rgb->getHeight() == depth->getHeight() && rgb->getWidth() == depth->getWidth());
    
    mRgb.reset(new common::KinectRgbImage());
    mRawDepth.reset(new common::KinectRawDepthImage());
    
    mRgb->Create(rgb->getWidth(), rgb->getHeight());
    mRawDepth->Create(depth->getWidth(), depth->getHeight());
    
    rgb->fillRGB(rgb->getWidth(), rgb->getHeight(), mRgb->GetArray().data());
    depth->fillDepthImageRaw(depth->getWidth(), depth->getHeight(), mRawDepth->GetArray().data());
    
    GenerateGrayImage();
    
    mFloatDepth.reset(new common::KinectFloatDepthImage());
    mFloatDepth->Create(depth->getWidth(), depth->getHeight());
    depth->fillDepthImage(depth->getWidth(), depth->getHeight(), mFloatDepth->GetArray().data());
    
    for(size_t i = 0; i < 10; i++)
    {
        size_t idxRaw = mRawDepth->ScanLineIndex(i);
        size_t idxFloat = mFloatDepth->ScanLineIndex(i);
        for(size_t j = 0; j < 10; j++)
        {
            mRawDepth->At(idxRaw + j) = 0;
            mFloatDepth->At(idxFloat + j) = std::numeric_limits<float>::quiet_NaN();
        }
    }
    
    mPC = mGrabber->GeneratePC(rgb, depth);
    
    FinishCycle();
    
    //Since this interface does not call ThreadEntry, so we have to call these signals here.
    mOnFinishSignal();
}

Eigen::Isometry3f KinectDeviceReader::GetCurrentGroundTruth()
{
    throw std::runtime_error("(KinectDeviceReader::GetCurrentGroundTruth) No ground truth information is provided.");
    return Eigen::Isometry3f::Identity();
}

bool KinectDeviceReader::ProvidesGroundTruth()
{
    return false; // For now there is not way to provide ground truth for a kinect device.
}

} // end namespace utils
} // end namespace KSRobot

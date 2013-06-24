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

KinectDeviceReader::KinectDeviceReader(const std::string& name) : common::KinectInterface(name)
{
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
    
    fn = boost::bind(&KinectDeviceReader::RGBDCallback, this, _1, _2, _3);
    mGrabber->registerCallback(fn);
}

bool KinectDeviceReader::ContinueExecution() const
{
    assert(mGrabber);
    return mGrabber->isRunning();
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
    assert(rgb->getHeight() == depth->getHeight() && rgb->getWidth() == depth->getWidth());
    
    LockData();
    
    mRgb.reset(new common::KinectRgbImage());
    mRawDepth.reset(new common::KinectRawDepthImage());
    mFloatDepth.reset(new common::KinectFloatDepthImage());
    
    mRgb->Create(rgb->getWidth(), rgb->getHeight());
    mFloatDepth->Create(depth->getWidth(), depth->getHeight());
    mRawDepth->Create(depth->getWidth(), depth->getHeight());
    
    rgb->fillRGB(rgb->getHeight(), rgb->getWidth(), mRgb->GetArray().data());
    //depth->fillDepthImage(depth->getHeight(), depth->getWidth(), mFloatDepth->GetArray().data());
    depth->fillDepthImageRaw(depth->getHeight(), depth->getWidth(), mRawDepth->GetArray().data());
    depth->fillDepthImage(depth->getHeight(), depth->getWidth(), mFloatDepth->GetArray().data());
    
    mPC = mGrabber->GeneratePC(rgb, depth);
    
    IncrementCycle();
    UnlockData();
}

Eigen::Isometry3d KinectDeviceReader::GetCurrentGroundTruth()
{
    throw std::runtime_error("(KinectDeviceReader::GetCurrentGroundTruth) No ground truth information is provided.");
    return Eigen::Isometry3d::Identity();
}

bool KinectDeviceReader::ProvidesGroundTruth()
{
    return false;
}

} // end namespace utils
} // end namespace KSRobot

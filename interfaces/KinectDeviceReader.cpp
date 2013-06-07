#include <interfaces/KinectDeviceReader.h>
#include <iostream>

namespace KSRobot
{
namespace interfaces
{

//TODO: SOlve bugs in pcl about inline virtual.
class GrabberHelper : public pcl::OpenNIGrabber
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

KinectDeviceReader::KinectDeviceReader(common::ProgramOptions::Ptr po, const std::string& name) : common::KinectInterface(po, name)
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
    LockData();
    
    mRgb->Create(rgb->getWidth(), rgb->getHeight());
    mFloatDepth->Create(depth->getWidth(), depth->getHeight());
    mRawDepth->Create(depth->getWidth(), depth->getHeight());
    
    rgb->fillRGB(rgb->getHeight(), rgb->getWidth(), mRgb->GetArray().data());
    depth->fillDepthImage(depth->getHeight(), depth->getWidth(), mFloatDepth->GetArray().data());
    depth->fillDepthImageRaw(depth->getHeight(), depth->getWidth(), mRawDepth->GetArray().data());
    
    mPC = mGrabber->GeneratePC(rgb, depth);
    
    UnlockData();
}

} // end namespace utils
} // end namespace KSRobot

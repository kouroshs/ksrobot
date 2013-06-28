#ifndef KINECTDEVICEREADER_H
#define KINECTDEVICEREADER_H

#include <common/KinectInterface.h>

namespace openni_wrapper
{
    class Image;
    class DepthImage;
};

namespace KSRobot
{
namespace interfaces
{

class KinectDeviceReader : public common::KinectInterface
{
public:
    typedef boost::shared_ptr<KinectDeviceReader>               Ptr;
    typedef boost::shared_ptr<const KinectDeviceReader>         ConstPtr;
    
    KinectDeviceReader();
    virtual ~KinectDeviceReader();
    
    virtual void                        Initialize(const std::string& device);
    virtual void                        Start();
    virtual void                        Stop();
    virtual bool                        ContinueExecution() const;
    
    virtual bool                        ProvidesGroundTruth();
    virtual Eigen::Isometry3d           GetCurrentGroundTruth();
    
    virtual bool                        RunSingleCycle() { return false; }
    
private:
   
    void                                RGBDCallback(const boost::shared_ptr<openni_wrapper::Image>&, 
                                                     const boost::shared_ptr<openni_wrapper::DepthImage>&, 
                                                     float invFocalLength);
private:
    class GrabberHelper;
    boost::shared_ptr<GrabberHelper>       mGrabber;
};

} // end namespace utils
} // end namespace KSRobot

#endif // KINECTDEVICEREADER_H

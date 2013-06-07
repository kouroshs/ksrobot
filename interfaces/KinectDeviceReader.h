#ifndef KINECTDEVICEREADER_H
#define KINECTDEVICEREADER_H

#include <common/KinectInterface.h>
#include <pcl/io/openni_grabber.h>

namespace KSRobot
{
namespace interfaces
{

class GrabberHelper;
class KinectDeviceReader : public common::KinectInterface
{
public:
    typedef boost::shared_ptr<KinectDeviceReader>               Ptr;
    typedef boost::shared_ptr<const KinectDeviceReader>         ConstPtr;
    
    KinectDeviceReader(common::ProgramOptions::Ptr po, const std::string& name);
    virtual ~KinectDeviceReader();
    
    virtual void                        Initialize(const std::string& device);
    virtual void                        Start();
    virtual void                        Stop();
    virtual bool                        ContinueExecution() const;
private:
    void                                RGBDCallback(const boost::shared_ptr<openni_wrapper::Image>&, 
                                                     const boost::shared_ptr<openni_wrapper::DepthImage>&, 
                                                     float invFocalLength);
private:
    boost::shared_ptr<GrabberHelper>       mGrabber;
};

} // end namespace utils
} // end namespace KSRobot

#endif // KINECTDEVICEREADER_H

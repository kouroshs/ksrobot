#ifndef KINECTINTERFACE_H
#define KINECTINTERFACE_H

#include <common/ProgramOptions.h>
#include <common/kinect/KinectImage.h>


#include <boost/function.hpp>
#include <boost/signals2.hpp>

#include <pcl/point_cloud.h> //basic pcl includes
#include <pcl/point_types.h>

namespace KSRobot
{
namespace common
{

typedef pcl::PointCloud<pcl::PointXYZRGBA>                                  KinectPointCloud;

class KinectInterface
{
public:
    typedef boost::shared_ptr<KinectInterface>                                  Ptr;
    typedef boost::shared_ptr<const KinectInterface>                            ConstPtr;
    
    typedef void (PointCloudReceiverFn)(const KinectPointCloud::ConstPtr&);
    typedef void (RGBDReceiverFn)(KinectRgbImage::Ptr rgb, KinectFloatDepthImage::Ptr depth);
    typedef void (RGBDRawReceiverFn)(KinectRgbImage::Ptr rgb, KinectRawDepthImage::Ptr depth);
public:
    KinectInterface(ProgramOptions::Ptr po);
    virtual ~KinectInterface();

    virtual void                        Initialize(const std::string& device) = 0;
    virtual void                        Start() = 0;
    virtual void                        Stop() = 0;
    virtual bool                        IsRunning() = 0;
    virtual boost::signals2::connection RegisterRGBDRawCallback(boost::function<RGBDRawReceiverFn> fn) = 0;
    virtual boost::signals2::connection RegisterRGBDFloatCallback(boost::function<RGBDReceiverFn> fn) = 0;
    virtual boost::signals2::connection RegisterPointCloudCallback(boost::function<PointCloudReceiverFn> fn) = 0;
protected:
    ProgramOptions::Ptr                 mPO;
};

} // end namespace utils
} // end namespace KSRobot


#endif // KINECTINTERFACE_H

#ifndef KINECTINTERFACE_H
#define KINECTINTERFACE_H

#include <common/Interface.h>
#include <common/KinectImage.h>


#include <boost/function.hpp>
#include <boost/signals2.hpp>

#include <pcl/point_cloud.h> //basic pcl includes
#include <pcl/point_types.h>

namespace KSRobot
{
namespace common
{

typedef pcl::PointCloud<pcl::PointXYZRGBA>                                  KinectPointCloud;

class KinectInterface : public Interface
{
public:
    
    struct CameraParameters
    {
        int                 Width;
        int                 Height;
        float               FocalX;
        float               FocalY;
        float               CenterX;
        float               CenterY;
        float               K1;
        float               K2;
        float               K3;
        float               P1;
        float               P2;
    };
    
    
    typedef boost::shared_ptr<KinectInterface>                                  Ptr;
    typedef boost::shared_ptr<const KinectInterface>                            ConstPtr;
    
    KinectInterface(ProgramOptions::Ptr po, const std::string& name);
    virtual ~KinectInterface();

    virtual void                                Initialize(const std::string& device) = 0;
    
    //NOTE: Before calling any of these functions, you have to call LockData()
    // Also you should make a copy of the data.
    inline KinectPointCloud::ConstPtr           GetPointCloud() const;
    inline KinectRgbImage::ConstPtr             GetRgbImage() const;
    inline KinectRawDepthImage::ConstPtr        GetRawDepthImage() const;
    inline KinectFloatDepthImage::ConstPtr      GetFloatDepthImage() const;
    
    inline CameraParameters                     GetCameraParams() const;
protected:
    KinectPointCloud::Ptr                       mPC;
    KinectRgbImage::Ptr                         mRgb;
    KinectRawDepthImage::Ptr                    mRawDepth;
    KinectFloatDepthImage::Ptr                  mFloatDepth;
    
    CameraParameters                            mParams;
    
    //TODO: Put sth for devisor, for example 5000 for datasetreader
    //TODO: Add camera params to this class
};

inline KinectPointCloud::ConstPtr KinectInterface::GetPointCloud() const
{
    return mPC;
}

inline KinectRgbImage::ConstPtr KinectInterface::GetRgbImage() const
{
    return mRgb;
}

inline KinectRawDepthImage::ConstPtr KinectInterface::GetRawDepthImage() const
{
    return mRawDepth;
}

inline KinectFloatDepthImage::ConstPtr KinectInterface::GetFloatDepthImage() const
{
    return mFloatDepth;
}

inline KinectInterface::CameraParameters KinectInterface::GetCameraParams() const
{
    return mParams;
}


} // end namespace utils
} // end namespace KSRobot


#endif // KINECTINTERFACE_H

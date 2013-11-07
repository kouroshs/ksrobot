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

// #ifdef KSROBOT_USE_DOUBLE_PRECISION_MATH
//     typedef Eigen::Isometry3d       IsometryTransform
// #else
//     typedef Eigen::Isometry3f       IsometryTransform
// #endif //KSROBOT_USE_DOUBLE_PRECISION_MATH

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
    
    KinectInterface();
    virtual ~KinectInterface();

    virtual void                                ReadSettings(ProgramOptions::Ptr po);
    virtual void                                Initialize(const std::string& device) = 0;
    
    //NOTE: Before calling any of these functions, you have to call LockData()
    // Also you should make a copy of the data.
    inline KinectPointCloud::ConstPtr           GetPointCloud() const;
    inline KinectRgbImage::ConstPtr             GetRgbImage() const;
    inline KinectGrayImage::ConstPtr            GetGrayImage() const;
    inline KinectRawDepthImage::ConstPtr        GetRawDepthImage() const;
    inline KinectFloatDepthImage::ConstPtr      GetFloatDepthImage() const;
    
    inline CameraParameters                     GetCameraParams() const;
    //TODO: Implement this
    inline float                                UnitsPerMeter() const { return 0; }
    
    virtual bool                                ProvidesGroundTruth() = 0;
    virtual Eigen::Isometry3f                   GetCurrentGroundTruth() = 0;
    
    bool                                        Get3DPosition(int rgbX, int rgbY, pcl::PointXYZ& ret) const;
    bool                                        Get3DPosition(int rgbX, int rgbY, int depthInCM, pcl::PointXYZ& ret) const;

    bool                                        Get3DPosition(int rgbX, int rgbY, Eigen::Vector3f& ret) const;
    bool                                        Get3DPosition(int rgbX, int rgbY, int depthInCM, Eigen::Vector3f& ret) const;
    
    
    static KinectPointCloud::Ptr                GeneratePointCloudFromImages(KinectRgbImage::Ptr rgb, 
                                                                   KinectRawDepthImage::Ptr depth, 
                                                                   const CameraParameters& camParams);
    
    static pcl::PointCloud<pcl::PointXYZ>::Ptr  GeneratePointCloudFromDepth(KinectRawDepthImage::Ptr depth, const CameraParameters& camParams);
protected:
    void                                        GenerateGrayImage();
    
protected:
    KinectPointCloud::Ptr                       mPC;
    KinectRgbImage::Ptr                         mRgb;
    KinectGrayImage::Ptr                        mGray;
    KinectRawDepthImage::Ptr                    mRawDepth;
    KinectFloatDepthImage::Ptr                  mFloatDepth;
    
    float                                       mRedCoef;
    float                                       mGreenCoef;
    float                                       mBlueCoef;
    CameraParameters                            mParams;
    CLASS_DEF_PYEXPORT;
};

inline KinectPointCloud::ConstPtr KinectInterface::GetPointCloud() const
{
    return mPC;
}

inline KinectRgbImage::ConstPtr KinectInterface::GetRgbImage() const
{
    return mRgb;
}

inline KinectGrayImage::ConstPtr KinectInterface::GetGrayImage() const
{
    return mGray;
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

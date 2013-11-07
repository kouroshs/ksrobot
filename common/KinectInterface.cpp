#include <common/KinectInterface.h>
#include <limits>

namespace KSRobot
{
namespace common
{

KinectInterface::KinectInterface() : Interface(), mPC(new KinectPointCloud()),
    mRgb(new KinectRgbImage()), mRawDepth(new KinectRawDepthImage()), mFloatDepth(new KinectFloatDepthImage()),
    mRedCoef(0.2125), mGreenCoef(0.7154), mBlueCoef(0.0721)
{
    memset(&mParams, 0, sizeof(mParams));
}

KinectInterface::~KinectInterface()
{
}

void KinectInterface::ReadSettings(ProgramOptions::Ptr po)
{
    Interface::ReadSettings(po);
    
    mRedCoef    = po->GetDouble("RGB2Gray.RedCoef", 0.2125);
    mGreenCoef  = po->GetDouble("RGB2Gray.GreenCoef", 0.7154);
    mBlueCoef   = po->GetDouble("RGB2Gray.BlueCoef", 0.0721);
}

void KinectInterface::GenerateGrayImage()
{
    // Generates the gray image after rgb image has been captured.
    mGray.reset(new KinectGrayImage(mRgb->GetWidth(), mRgb->GetHeight()));
    const KinectRgbImage::ArrayType& src = mRgb->GetArray();
    KinectGrayImage::ArrayType& dst = mGray->GetArray();
    const size_t count = mGray->GetWidth() * mGray->GetHeight();
    
    for(size_t i = 0; i < count; i++)
    {
        const size_t startIndex = i * 3;
        dst[i] = (int)roundf(mRedCoef * src[startIndex] + 
                                mGreenCoef * src[startIndex + 1] + 
                                mBlueCoef * src[startIndex + 2]);
    }
}

KinectPointCloud::Ptr KinectInterface::GeneratePointCloudFromImages(KinectRgbImage::Ptr rgb, 
                                                          KinectRawDepthImage::Ptr depth, 
                                                          const KinectInterface::CameraParameters& camParams)
{
    assert(rgb->GetHeight() == depth->GetHeight() && 
           rgb->GetWidth() == depth->GetWidth());
    
    const unsigned short int iShadowValue = 0;
    const unsigned short int iNoSampleValue = 0;
    
    KinectPointCloud::Ptr cloud(new KinectPointCloud);
    cloud->header.frame_id = "KinectPointCloudFrame";
    cloud->height = rgb->GetHeight();
    cloud->width = rgb->GetWidth();
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
    cloud->sensor_orientation_.setIdentity();
    cloud->sensor_origin_.fill(0.0f);
    
    float constant_x = 1.0f / camParams.FocalX;
    float constant_y = 1.0f / camParams.FocalY;
    float centerX = camParams.CenterX;
    float centerY = camParams.CenterY;
    
    const float bad_point = std::numeric_limits<float>::quiet_NaN();
    
    int value_idx = 0;
    int rgb_idx = 0;
    int point_idx = 0;
    
    // Fill in depth and rgb values
    for(int v = 0; v < depth->GetHeight(); v++)
    {
        for(int u = 0; u < depth->GetWidth(); u++, point_idx++, value_idx++, rgb_idx += 3)
        {
            pcl::PointXYZRGBA& point = cloud->points[point_idx];
            point.r = rgb->At(rgb_idx);
            point.g = rgb->At(rgb_idx + 1);
            point.b = rgb->At(rgb_idx + 2);
            point.a = 0xFF;
            
            if( depth->At(value_idx) != 0 )
            {
                //valid point
                point.z = depth->At(value_idx) * 0.001f;
                point.x = (u - centerX) * point.z * constant_x;
                point.y = (v - centerY) * point.z * constant_y;
            }
            else
            {
                point.x = point.y = point.z = bad_point;
            }
        }
    }
    
    cloud->sensor_origin_.setZero();
    cloud->sensor_orientation_.w() = 0.0f;
    cloud->sensor_orientation_.x() = 1.0f;
    cloud->sensor_orientation_.y() = 0.0f;
    cloud->sensor_orientation_.z() = 0.0f;
    
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr KinectInterface::GeneratePointCloudFromDepth(KinectRawDepthImage::Ptr depth, 
                                                                                 const KinectInterface::CameraParameters& camParams)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->header.frame_id = "KinectPointCloudFrame";
    cloud->height = depth->GetHeight();
    cloud->width = depth->GetWidth();
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
    cloud->sensor_orientation_.setIdentity();
    cloud->sensor_origin_.fill(0.0f);
    
    float constant_x = 1.0f / camParams.FocalX;
    float constant_y = 1.0f / camParams.FocalY;
    float centerX = camParams.CenterX;
    float centerY = camParams.CenterY;
    
    const float bad_point = std::numeric_limits<float>::quiet_NaN();
    
    int value_idx = 0;
    int rgb_idx = 0;
    int point_idx = 0;
    
    // Fill in depth and rgb values
    for(int v = 0; v < depth->GetHeight(); v++)
    {
        for(int u = 0; u < depth->GetWidth(); u++, point_idx++, value_idx++, rgb_idx += 3)
        {
            pcl::PointXYZ& point = cloud->points[point_idx];
            
            if( depth->At(value_idx) != 0 )
            {
                //valid point
                point.z = depth->At(value_idx) * 0.001f;
                point.x = (u - centerX) * point.z * constant_x;
                point.y = (v - centerY) * point.z * constant_y;
            }
            else
            {
                point.x = point.y = point.z = bad_point;
            }
        }
    }
    
    cloud->sensor_origin_.setZero();
    cloud->sensor_orientation_.w() = 0.0f;
    cloud->sensor_orientation_.x() = 1.0f;
    cloud->sensor_orientation_.y() = 0.0f;
    cloud->sensor_orientation_.z() = 0.0f;
    
    return cloud;
}

bool KinectInterface::Get3DPosition(int rgbX, int rgbY, pcl::PointXYZ& ret) const
{
    return Get3DPosition(rgbX, rgbY, mFloatDepth->At(rgbY * mFloatDepth->GetWidth() + rgbX), ret);
}

bool KinectInterface::Get3DPosition(int rgbX, int rgbY, int depth, pcl::PointXYZ& p) const
{
    if( depth == 0 )
    {
        p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN();
        return false;
    }
    else
    {
        p.z = depth * 0.001f;
        p.x = (rgbX - mParams.CenterX) * p.z / mParams.FocalX;
        p.y = (rgbY - mParams.CenterY) * p.z / mParams.FocalY;
        return true;
    }
}

bool KinectInterface::Get3DPosition(int rgbX, int rgbY, Eigen::Vector3f& p) const
{
    return Get3DPosition(rgbX, rgbY, mFloatDepth->At(rgbY * mFloatDepth->GetWidth() + rgbX), p);
}

bool KinectInterface::Get3DPosition(int rgbX, int rgbY, int depthInCM, Eigen::Vector3f& p) const
{
    if( depthInCM == 0 )
    {
        p[0] = p[1] = p[2] = std::numeric_limits<float>::quiet_NaN();
        return false;
    }
    else
    {
        p[2] = depthInCM * 0.001f;
        p[0] = (rgbX - mParams.CenterX) * p[2] / mParams.FocalX;
        p[1] = (rgbY - mParams.CenterY) * p[2] / mParams.FocalY;
        return true;
    }
}



} // end namespace utils
} // end namespace KSRobot

/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2013  <copyright holder> <email>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 * 
 */

#ifndef PCLUTILS_H
#define PCLUTILS_H

#include <common/KinectInterface.h>
#include <pcl/registration/gicp.h>

namespace KSRobot
{
namespace common
{

class PCLUtils
{
public:
    static inline KinectPointCloud::Ptr    ApplyVoxelGrid(KinectPointCloud::ConstPtr in, float leaf_size);
    static inline KinectPointCloud::Ptr    ApplyPassThrough(KinectPointCloud::ConstPtr in, const std::string& field_name, 
                                                            float range_min, float range_max, bool set_negative = false);
    
    static void                     ApplyVoxelGrid(KinectPointCloud& out, KinectPointCloud::ConstPtr in, float leaf_size);
    static void                     ApplyPassThrough(KinectPointCloud& out, KinectPointCloud::ConstPtr in, const std::string& field_name, 
                                                     float range_min, float range_max, bool set_negative = false);
    
    static void                     ConvertPointCloud(KinectPointCloud::ConstPtr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out);
    static void                     DownsampleOrganized(KinectPointCloud::ConstPtr in, KinectPointCloud::Ptr out, size_t ds_rate);
    static void                     DownsampleOrganized(KinectPointCloud::ConstPtr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, size_t ds_rate);
    static void                     DownsampleOrganized(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, 
                                                        size_t ds_rate);
    
    static void                     ComputeNormals(KinectPointCloud::ConstPtr in, pcl::PointCloud<pcl::Normal>::Ptr normals);
    static void                     ComputeNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in, pcl::PointCloud<pcl::Normal>::Ptr normals);
    
    static void                     NormalSpaceSampling(KinectPointCloud::Ptr out, KinectPointCloud::ConstPtr in,
                                                        pcl::PointCloud<pcl::Normal>::ConstPtr normals, int final_size = 3000,
                                                        int bins = 50);

    static void                     NormalSpaceSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr out, pcl::PointCloud<pcl::PointXYZ>::ConstPtr in,
                                                        pcl::PointCloud<pcl::Normal>::ConstPtr normals, int final_size = 3000,
                                                        int bins = 50);
    
};

inline KinectPointCloud::Ptr PCLUtils::ApplyVoxelGrid(KinectPointCloud::ConstPtr in, float leaf_size)
{
    KinectPointCloud::Ptr out(new KinectPointCloud);
    ApplyVoxelGrid(*out, in, leaf_size);
    return out;
}

inline KinectPointCloud::Ptr PCLUtils::ApplyPassThrough(KinectPointCloud::ConstPtr in, const std::string& field_name, 
                                                 float range_min, float range_max, bool set_negative)
{
    KinectPointCloud::Ptr out(new KinectPointCloud);
    ApplyPassThrough(*out, in, field_name, range_min, range_max, set_negative);
    return out;
}


class GICP
{
public:
    GICP();
    ~GICP();
    
    void                                        SetInputSource(KinectPointCloud::ConstPtr source);
    void                                        SetInputTraget(KinectPointCloud::ConstPtr target);
    
    void                                        SetInputSource(pcl::PointCloud<pcl::PointXYZ>::ConstPtr source);
    void                                        SetInputTraget(pcl::PointCloud<pcl::PointXYZ>::ConstPtr target);
    
    inline void                                 SetDownsampleRate(int rate) { mDownsampleRate = rate; }
    
    void                                        ComputeTransform();
    
    inline void                                 SetTransformationEpsilon(float eps) { mICP.setTransformationEpsilon(eps); }
    inline void                                 SetMaxInterations(int max) { mICP.setMaximumIterations(max); }
    
    inline Eigen::Matrix4f                      GetFinalComputedTransform() const { return mTransform; }
    inline bool                                 Converged() const { return mConverged; }
private:
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>  mICP;
    pcl::PointCloud<pcl::PointXYZ>::Ptr         mSource;
    pcl::PointCloud<pcl::PointXYZ>::Ptr         mTarget;
    pcl::PointCloud<pcl::Normal>::Ptr           mSourceNormals;
    pcl::PointCloud<pcl::Normal>::Ptr           mTargetNormals;
    pcl::PointCloud<pcl::PointXYZ>::Ptr         mSourceDownsampled;
    pcl::PointCloud<pcl::PointXYZ>::Ptr         mTargetDownsampled;
    
    pcl::PointCloud<pcl::PointXYZ>              mFinalCloud;
    
    Eigen::Matrix4f                             mTransform;
    bool                                        mConverged;
    int                                         mDownsampleRate;
};

}
}

#endif // PCLUTILS_H

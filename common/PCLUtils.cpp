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

#include <common/PCLUtils.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/normal_space.h>
#include <limits>

using namespace KSRobot::common;

void PCLUtils::ConvertPointCloud(KinectPointCloud::ConstPtr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out)
{
    out->clear();
    out->width = in->width;
    out->height = in->height;
    out->sensor_orientation_ = in->sensor_orientation_;
    out->sensor_origin_ = in->sensor_origin_;
    out->header = in->header;
    
    out->resize(in->size());
    for(size_t i = 0; i < in->size(); i++)
        out->at(i).getVector4fMap() = in->at(i).getVector4fMap();
}

void PCLUtils::DownsampleOrganized(KinectPointCloud::ConstPtr in, KinectPointCloud::Ptr out, size_t ds_rate)
{
    out->clear();
    out->width = in->width / ds_rate;
    out->height = in->height / ds_rate;
    out->sensor_orientation_ = in->sensor_orientation_;
    out->sensor_origin_ = in->sensor_origin_;
    
    out->resize(out->width * out->height);
    KinectPointCloud::PointType initial_point;
    initial_point.rgba = 0;
    initial_point.getVector4fMap() = Eigen::Vector4f(0, 0, 0, 0);
    
    const float bad_point = std::numeric_limits<float>::quiet_NaN();
    const Eigen::Vector4f bad_position = Eigen::Vector4f(bad_point, bad_point, bad_point, bad_point);
    
    for(size_t i = 0; i < in->height; i += ds_rate)
    {
        size_t out_line_index = (i / ds_rate) * in->width;
        
        for(size_t j = 0; j < in->width; j+= ds_rate)
        {
            // now take average of all 
            size_t count = 0;
            Eigen::Vector4f pos(0, 0, 0, 0);
            float r = 0, g = 0, b = 0;
            for(size_t curr_i = i; curr_i < std::min(i + ds_rate, (size_t)in->height); curr_i++)
            {
                size_t in_start_index = curr_i * in->width;
                for(size_t curr_j = j; curr_j < std::min(j + ds_rate, (size_t)in->width); curr_j++)
                {
                    const KinectPointCloud::PointType& pt = in->at(curr_j + in_start_index);
                    if( pcl::isFinite<KinectPointCloud::PointType>(pt) )
                    {
                        count++;
                        pos += pt.getVector4fMap();
                        r += pt.r;
                        g += pt.g;
                        b += pt.b;
                    }
                }
            }
            
            KinectPointCloud::PointType& dst = out->at(out_line_index + j / ds_rate);
            
            if( count == 0 )
            {
                //invald point
                dst.getVector4fMap() = bad_position;
            }
            else
            {
                dst.getVector4fMap() = pos / count;
                dst.r = r / count;
                dst.g = g / count;
                dst.b = b / count;
            }
            
        }
    }
}

void PCLUtils::DownsampleOrganized(KinectPointCloud::ConstPtr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, size_t ds_rate)
{
    out->clear();
    out->width = in->width / ds_rate;
    out->height = in->height / ds_rate;
    out->sensor_orientation_ = in->sensor_orientation_;
    out->sensor_origin_ = in->sensor_origin_;
    
    out->resize(out->width * out->height);
    const pcl::PointXYZ initial_point(0, 0, 0);
    const float bad_point = std::numeric_limits<float>::quiet_NaN();
    const Eigen::Vector4f bad_position(bad_point, bad_point, bad_point, bad_point);
    
    for(size_t i = 0; i < in->height; i += ds_rate)
    {
        size_t out_line_index = (i / ds_rate) * in->width;
        
        for(size_t j = 0; j < in->width; j+= ds_rate)
        {
            // now take average of all 
            size_t count = 0;
            Eigen::Vector4f pos(0, 0, 0, 0);
            for(size_t curr_i = i; curr_i < std::min(i + ds_rate, (size_t)in->height); curr_i++)
            {
                size_t in_start_index = curr_i * in->width;
                for(size_t curr_j = j; curr_j < std::min(j + ds_rate, (size_t)in->width); curr_j++)
                {
                    const KinectPointCloud::PointType& pt = in->at(curr_j + in_start_index);
                    if( pcl::isFinite<KinectPointCloud::PointType>(pt) )
                    {
                        count++;
                        pos += pt.getVector4fMap();
                    }
                }
            }
            
            pcl::PointXYZ& dst = out->at(out_line_index + j / ds_rate);
            
            if( count == 0 )
                dst.getVector4fMap() = bad_position;
            else
                dst.getVector4fMap() = pos / count;
        }
    }
}

void PCLUtils::ComputeNormals(KinectPointCloud::ConstPtr in, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    pcl::IntegralImageNormalEstimation<KinectPointCloud::PointType, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(in);
    ne.compute(*normals);
}

void PCLUtils::ComputeNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(in);
    ne.compute(*normals);
}

void PCLUtils::NormalSpaceSampling(KinectPointCloud::Ptr out, KinectPointCloud::ConstPtr in, 
                                   pcl::PointCloud<pcl::Normal>::ConstPtr normals, int final_size, int bins)
{
    pcl::NormalSpaceSampling<KinectPointCloud::PointType, pcl::Normal> nss;
    nss.setBins(bins, bins, bins);
    nss.setSample(final_size);
    nss.setInputCloud(in);
    nss.setNormals(normals);
    nss.filter(*out);
}

void PCLUtils::NormalSpaceSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr out, pcl::PointCloud<pcl::PointXYZ>::ConstPtr in, 
                                   pcl::PointCloud<pcl::Normal>::ConstPtr normals, int final_size, int bins)
{
    pcl::NormalSpaceSampling<pcl::PointXYZ, pcl::Normal> nss;
    nss.setBins(bins, bins, bins);
    nss.setSample(final_size);
    nss.setInputCloud(in);
    nss.setNormals(normals);
    nss.filter(*out);
}



GICP::GICP()
{
    mSource.reset(new pcl::PointCloud<pcl::PointXYZ>);
    mTarget.reset(new pcl::PointCloud<pcl::PointXYZ>);
    mSourceDownsampled.reset(new pcl::PointCloud<pcl::PointXYZ>);
    mTargetDownsampled.reset(new pcl::PointCloud<pcl::PointXYZ>);
    mSourceNormals.reset(new pcl::PointCloud<pcl::Normal>);
    mTargetNormals.reset(new pcl::PointCloud<pcl::Normal>);
    mDownsampleRate = 2;
    mConverged = false;
}

GICP::~GICP()
{
}

void GICP::SetInputSource(KinectPointCloud::ConstPtr source)
{
    PCLUtils::DownsampleOrganized(source, mSource, mDownsampleRate);
    PCLUtils::ComputeNormals(mSource, mSourceNormals);
    PCLUtils::NormalSpaceSampling(mSourceDownsampled, mSource, mSourceNormals);
}

void GICP::SetInputTraget(KinectPointCloud::ConstPtr target)
{
    PCLUtils::DownsampleOrganized(target, mTarget, mDownsampleRate);
    PCLUtils::ComputeNormals(mTarget, mTargetNormals);
    PCLUtils::NormalSpaceSampling(mTargetDownsampled, mTarget, mTargetNormals);
}

void GICP::ComputeTransform()
{
    mICP.setInputSource(mSourceDownsampled);
    mICP.setInputTarget(mTargetDownsampled);
    mFinalCloud.clear();
    mICP.align(mFinalCloud);
    mConverged = mICP.hasConverged();
    mTransform = mICP.getFinalTransformation();
    mTransform = mTransform.inverse(); // TODO: Check this.
}


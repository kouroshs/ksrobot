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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <limits>

using namespace KSRobot::common;

void PCLUtils::ApplyPassThrough(KinectPointCloud& out, KinectPointCloud::ConstPtr in, const std::string& field_name,
                                float range_min, float range_max, bool set_negative)
{
    pcl::PassThrough<KinectPointCloud::PointType> pt;
    pt.setFilterFieldName(field_name);
    pt.setFilterLimits(range_min, range_max);
    pt.setFilterLimitsNegative(set_negative);
    pt.setInputCloud(in);
    pt.filter(out);
}

void PCLUtils::ApplyVoxelGrid(KinectPointCloud& out, KinectPointCloud::ConstPtr in, float leaf_size)
{
    pcl::VoxelGrid<KinectPointCloud::PointType> v;
    v.setLeafSize(leaf_size, leaf_size, leaf_size);
    v.setInputCloud(in);
    v.filter(out);
}

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

// I know, this is bad. but I don't know enough of pcl internals to reduce the code size yet.
void PCLUtils::DownsampleOrganized(KinectPointCloud::ConstPtr in, KinectPointCloud::Ptr out, size_t ds_rate)
{
    assert(ds_rate > 0 );
    if( ds_rate == 1 )
    {
        *out = *in;
        return;
    }
    
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
    
    size_t out_index_i = 0;
    for(size_t i = 0; i < in->height; i += ds_rate, out_index_i++)
    {
        size_t out_line_index = out_index_i * out->width;
        size_t out_index_j = 0;
        for(size_t j = 0; j < in->width; j += ds_rate, out_index_j++)
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
            
            KinectPointCloud::PointType& dst = out->at(out_line_index + out_index_j);
            
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
    assert(ds_rate > 0 );
    
    out->clear();
    out->width = in->width / ds_rate;
    out->height = in->height / ds_rate;
    out->sensor_orientation_ = in->sensor_orientation_;
    out->sensor_origin_ = in->sensor_origin_;
    
    out->resize(out->width * out->height);
    const pcl::PointXYZ initial_point(0, 0, 0);
    const float bad_point = std::numeric_limits<float>::quiet_NaN();
    const Eigen::Vector4f bad_position(bad_point, bad_point, bad_point, bad_point);
    
    size_t out_index_i = 0;
    for(size_t i = 0; i < in->height; i += ds_rate, out_index_i++)
    {
        size_t out_line_index = out_index_i * out->width;
        size_t out_index_j = 0;
        for(size_t j = 0; j < in->width; j+= ds_rate, out_index_j++)
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
            
            pcl::PointXYZ& dst = out->at(out_line_index + out_index_j);
            
            if( count == 0 )
                dst.getVector4fMap() = bad_position;
            else
                dst.getVector4fMap() = pos / count;
        }
    }
}

void PCLUtils::DownsampleOrganized(pcl::PointCloud< pcl::PointXYZ >::ConstPtr in, pcl::PointCloud< pcl::PointXYZ >::Ptr out, size_t ds_rate)
{
    assert(ds_rate > 0 );
    if( ds_rate == 1 )
    {
        *out = *in;
        return;
    }
    
    out->clear();
    out->width = in->width / ds_rate;
    out->height = in->height / ds_rate;
    out->sensor_orientation_ = in->sensor_orientation_;
    out->sensor_origin_ = in->sensor_origin_;
    
    out->resize(out->width * out->height);
    const pcl::PointXYZ initial_point(0, 0, 0);
    const float bad_point = std::numeric_limits<float>::quiet_NaN();
    const Eigen::Vector4f bad_position(bad_point, bad_point, bad_point, bad_point);
    
    size_t out_index_i = 0;
    for(size_t i = 0; i < in->height; i += ds_rate, out_index_i++)
    {
        size_t out_line_index = out_index_i * out->width;
        size_t out_index_j = 0;
        for(size_t j = 0; j < in->width; j += ds_rate, out_index_j++)
        {
            // now take average of all 
            size_t count = 0;
            Eigen::Vector4f pos(0, 0, 0, 0);
            for(size_t curr_i = i; curr_i < std::min(i + ds_rate, (size_t)in->height); curr_i++)
            {
                size_t in_start_index = curr_i * in->width;
                for(size_t curr_j = j; curr_j < std::min(j + ds_rate, (size_t)in->width); curr_j++)
                {
                    const pcl::PointXYZ& pt = in->at(curr_j + in_start_index);
                    if( pcl::isFinite<pcl::PointXYZ>(pt) )
                    {
                        count++;
                        pos += pt.getVector4fMap();
                    }
                }
            }
            
            pcl::PointXYZ& dst = out->at(out_line_index + out_index_j);
            
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

static void RemoveNans(pcl::PointCloud<pcl::PointXYZ>::Ptr p)
{    
    // I don't know why but this seems not to work, and does not remove nans compeletly, possibly a bug.
//     std::vector<int> tmp;
//     pcl::removeNaNFromPointCloud(*p, *p, tmp);
    pcl::PointCloud<pcl::PointXYZ> tmp;
    tmp.header = p->header;
    tmp.reserve(p->size());
    for(size_t i = 0; i < p->size(); i++)
        if( pcl::isFinite(p->at(i)) )
            tmp.push_back(p->at(i));
    
    *p = tmp;
}

void GICP::SetInputSource(KinectPointCloud::ConstPtr source)
{
    PCLUtils::DownsampleOrganized(source, mSource, mDownsampleRate);
    PCLUtils::ComputeNormals(mSource, mSourceNormals);
    PCLUtils::NormalSpaceSampling(mSourceDownsampled, mSource, mSourceNormals);
    RemoveNans(mSourceDownsampled);
}

void GICP::SetInputTraget(KinectPointCloud::ConstPtr target)
{
    PCLUtils::DownsampleOrganized(target, mTarget, mDownsampleRate);
    PCLUtils::ComputeNormals(mTarget, mTargetNormals);
    PCLUtils::NormalSpaceSampling(mTargetDownsampled, mTarget, mTargetNormals);
    RemoveNans(mTargetDownsampled);
}

void GICP::SetInputSource(pcl::PointCloud< pcl::PointXYZ >::ConstPtr source)
{
    PCLUtils::DownsampleOrganized(source, mSource, mDownsampleRate);
    PCLUtils::ComputeNormals(mSource, mSourceNormals);
    PCLUtils::NormalSpaceSampling(mSourceDownsampled, mSource, mSourceNormals);
    RemoveNans(mSourceDownsampled);
}

void GICP::SetInputTraget(pcl::PointCloud< pcl::PointXYZ >::ConstPtr target)
{
    PCLUtils::DownsampleOrganized(target, mTarget, mDownsampleRate);
    PCLUtils::ComputeNormals(mTarget, mTargetNormals);
    PCLUtils::NormalSpaceSampling(mTargetDownsampled, mTarget, mTargetNormals);
    RemoveNans(mTargetDownsampled);
}

void GICP::ComputeTransform()
{
    mICP.setInputSource(mSourceDownsampled);
    mICP.setInputTarget(mTargetDownsampled);
    
    mFinalCloud.clear();
    mICP.align(mFinalCloud);
    mConverged = mICP.hasConverged();
    mTransform = mICP.getFinalTransformation();
    mTransform = mTransform.inverse().eval(); // TODO: Check this.
}


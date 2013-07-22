/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2013  Kourosh <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <interfaces/OctomapInterface.h>

#include <boost/algorithm/string.hpp>

#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>


#define DEFAULT_USE_COLOR                           false
#define DEFAULT_MAP_RESOLUTION                      0.1
#define DEFAULT_MAX_RANGE                           4.0
#define DEFAULT_APPLY_VOXEL_GRID                    true
#define DEFAULT_VOXEL_GRID_RESOLUTION               0.05
#define DEFAULT_LAZY_EVAL                           false


namespace KSRobot
{
namespace interfaces
{

OctomapInterface::OctomapInterface() : common::MappingInterface(), mUseColor(DEFAULT_USE_COLOR), mMapResolution(DEFAULT_MAP_RESOLUTION),
    mMaxRange(DEFAULT_MAX_RANGE), mApplyVoxelGrid(DEFAULT_APPLY_VOXEL_GRID), mVoxelGridResolution(DEFAULT_MAP_RESOLUTION),
    mLazyEval(DEFAULT_LAZY_EVAL), mEnableHeightFiltering(false), mHeightFilterAxis(1), mHeightFilterMaxValue(0), mHeightFilterMinValue(0)
{
    mGroundFilter.Enabled = false; // disable it, no need to set defaults
    mLocalTransformInfo.Enabled = false;
    mPassThrough.Enabled = false;
}

OctomapInterface::~OctomapInterface()
{
}

void OctomapInterface::Initialize()
{
    common::MappingInterface::Initialize();    
    mOctree.reset(new octomap::ColorOcTree(mMapResolution));
    
    if( mGroundFilter.Enabled && ! mLocalTransformInfo.Enabled )
        throw std::runtime_error("(OctomapInterface::Initialize) Cannot enable ground filter while local transform filter is disabled.");
    
    if( mLocalTransformInfo.Enabled )
    {
        Eigen::AngleAxisf r1, r2;
        r1 = r2 = Eigen::AngleAxisf::Identity();
        if( mLocalTransformInfo.FirstRotationAxis != -1 )
        {
            Eigen::Vector3f axis(0, 0, 0);
            axis[mLocalTransformInfo.FirstRotationAxis] = 1;
            r1 = Eigen::AngleAxisf(mLocalTransformInfo.FirstRotationDegree, axis);
        }
        if( mLocalTransformInfo.SecondRotationAxis != -1 )
        {
            Eigen::Vector3f axis(0, 0, 0);
            axis[mLocalTransformInfo.SecondRotationAxis] = 1;
            r2 = Eigen::AngleAxisf(mLocalTransformInfo.SecondRotationDegree, axis);
        }
        
        mCameraTransformCached.setIdentity();
        mCameraTransformCached.linear() = (r2 * r1).toRotationMatrix();
        if( mLocalTransformInfo.RobotHeightAxis != -1 )
            mCameraTransformCached.translation()[mLocalTransformInfo.RobotHeightAxis] = mLocalTransformInfo.RobotHeight;
    }
}

void OctomapInterface::SaveToFile(const std::string& filename)
{
    mOctree->write(filename);
}

static inline int GetAxisFromString(std::string& axis)
{
    boost::to_lower(axis);
    if( axis == "x" )
        return 0;
    else if( axis == "y" )
        return 1;
    else if( axis == "z" )
        return 2;
    else if( axis == "none" )
        return -1;
    else
        throw std::runtime_error("(OctomapInterface::ReadSettings) Invalid axis is provided.");
}

void OctomapInterface::ReadSettings(common::ProgramOptions::Ptr po)
{
    mUseColor = po->GetBool("UseColor", DEFAULT_USE_COLOR);
    mMapResolution = po->GetDouble("MapResolution", DEFAULT_MAP_RESOLUTION);
    mMaxRange = po->GetDouble("MaxRange", DEFAULT_MAX_RANGE);
    mLazyEval = po->GetBool("LazyEval", DEFAULT_LAZY_EVAL);
    mApplyVoxelGrid = po->GetBool("VoxelGrid.Enabled", DEFAULT_APPLY_VOXEL_GRID);
    mVoxelGridResolution = po->GetDouble("VoxelGrid.Resolution", DEFAULT_VOXEL_GRID_RESOLUTION);
    
    mEnableHeightFiltering = po->GetBool("HeightFilter.Enabled", false);
    std::string axis = po->GetString("HeightFilter.Axis", std::string("y"));
    mHeightFilterAxis = GetAxisFromString(axis);
    assert(mHeightFilterAxis != -1);
    
    mHeightFilterMaxValue = po->GetDouble("HeightFilter.MaxValue", 0.0);
    mHeightFilterMinValue = po->GetDouble("HeightFilter.MinValue", 0.0);
    
    mGroundFilter.Enabled = po->GetBool("GroundFilter.Enabled", false);
    mGroundFilter.Axis = po->GetInt("GroundFilter.Axis", 1);
    mGroundFilter.Distance = po->GetDouble("GroundFilter.Distance", 0.04);
    mGroundFilter.PlaneDistance = po->GetDouble("GroundFilter.PlaneDistance", 0.04);
    mGroundFilter.Angle = po->GetDouble("GroundFilter.Angle", 0.15);
    
    common::ProgramOptions::Ptr p = po->StartNode("LocalTransform");
    mLocalTransformInfo.Enabled = p->GetBool("Enabled", false);
    mLocalTransformInfo.RobotHeight = p->GetDouble("RobotHeight", 0.0);
    axis = p->GetString("RobotHeightAxis", std::string("z"));
    mLocalTransformInfo.RobotHeightAxis = GetAxisFromString(axis);
    assert(mLocalTransformInfo.RobotHeightAxis != -1);
    
    axis = p->GetString("FirstRotationAxis", std::string("none"));
    mLocalTransformInfo.FirstRotationAxis = GetAxisFromString(axis);
    mLocalTransformInfo.FirstRotationDegree = p->GetDouble("FirstRotationDegree", 0.0);
    axis = p->GetString("SecondRotationAxis", std::string("none"));
    mLocalTransformInfo.SecondRotationAxis = GetAxisFromString(axis);
    mLocalTransformInfo.SecondRotationDegree = p->GetDouble("SecondRotationDegree", 0.0);
    
    p = po->StartNode("PassThroughFilter");
    mPassThrough.Enabled = p->GetBool("Enabled", true);
    mPassThrough.LimitMin = p->GetDouble("LimitMin", 0.5);
    mPassThrough.LimitMax = p->GetDouble("LimitMax", 4.0);
    
    Initialize();
}

bool OctomapInterface::RunSingleCycle()
{
    common::MappingInterface::MapInfo mi;
    int count = 0;
    // I moved these filter defenitions out of the loop as a small optimization, nothing more.
    pcl::VoxelGrid<common::KinectPointCloud::PointType> voxelGrid;
    if( mApplyVoxelGrid )
        voxelGrid.setLeafSize(mVoxelGridResolution, mVoxelGridResolution, mVoxelGridResolution);

    pcl::PassThrough<common::KinectPointCloud::PointType> pass;
    if( mPassThrough.Enabled )
    {
        pass.setFilterFieldName("z");
        pass.setFilterLimits(mPassThrough.LimitMin, mPassThrough.LimitMax);
    }

    common::KinectPointCloud pc; // a useful temp var
    
    while( mKeyframesQueue.try_pop(mi) )
    {
        common::Interface::ScopedLock locker(this);
        count++;
        
        if( mApplyVoxelGrid )
        {
            voxelGrid.setInputCloud(mi.PointCloud);
            voxelGrid.filter(*mFilteredCloud);
        }
        else
        {
            // copy this pointcloud! not the best performance, but causes better and cleaner program
            // since we usually should use voxel grids and pass throughs, then this is no problem
            *mFilteredCloud = *(mi.PointCloud);
        }
        
        if( mPassThrough.Enabled )
        {
            pass.setInputCloud(mFilteredCloud);
            pass.filter(pc);
            *mFilteredCloud = pc;
        }
        
        if( mLocalTransformInfo.Enabled )
            pcl::transformPointCloud(*mFilteredCloud, *mFilteredCloud, mCameraTransformCached);
        
        if( mGroundFilter.Enabled && ExtractGroundPlane(*mFilteredCloud, *mGroundPC, *mNonGroundPC) )
            *mFilteredCloud = *mGroundPC;
        
        ConvertInternal(mFilteredCloud, mi.Transform);
        UpdateMap(mFilteredCloud);
        
        FinishCycle();
    }
    return count != 0;
}

void OctomapInterface::ConvertInternal(common::KinectPointCloud::ConstPtr pc, const Eigen::Isometry3f& transform)
{
    mPoints.clear();
    if( !mUseColor )
    {
        for(size_t i = 0; i < pc->size(); i++)
        {
            const common::KinectPointCloud::PointType& p = pc->at(i);
            if( pcl::isFinite(p) )
                mPoints.push_back(p.x, p.y, p.z);
        }
    }
    else
    {
        mIndexes.clear();
        mIndexes.reserve(pc->size());
        for(size_t i = 0; i < pc->size(); i++)
        {
            const common::KinectPointCloud::PointType& p = pc->at(i);
            if( pcl::isFinite(p) )
            {
                mPoints.push_back(p.x, p.y, p.z);
                mIndexes.push_back(i);
            }
        }
    }
    
    const Eigen::Quaternionf eigen_rot(transform.linear());
    const Eigen::Vector3f translate = transform.translation();
    octomath::Quaternion rot(eigen_rot.w(), eigen_rot.x(), eigen_rot.y(), eigen_rot.z());
    mPoints.transform(octomath::Pose6D(octomath::Vector3(translate[0], translate[1], translate[2]), rot));
}

void OctomapInterface::UpdateMap(common::KinectPointCloud::ConstPtr pc)
{
    octomap::KeySet occupied, free;
    mOctree->computeUpdate(mPoints, octomap::point3d(0,0,0), free, occupied, mMaxRange);
    
    for(octomap::KeySet::iterator iter = free.begin(); iter != free.end(); iter++)
        mOctree->updateNode(*iter, false, mLazyEval);
    
    for(octomap::KeySet::iterator iter = occupied.begin(); iter != free.end(); iter++)
        mOctree->updateNode(*iter, true, mLazyEval);
    
    if( mUseColor )
    {
        int index = 0;
        for(octomap::KeySet::iterator iter = occupied.begin(); iter != free.end(); iter++, index++)
        {
            const common::KinectPointCloud::PointType& p = pc->at(mIndexes[index]);
            mOctree->integrateNodeColor(*iter, p.r, p.g, p.b);
        }
    }
}

void OctomapInterface::ConvertToOccupancyGrid(common::OccupancyMap2D& map, int center_i, int center_j, bool lock_interface)
{
    if( lock_interface )
        LockData();
    
    const unsigned int max_depth = mOctree->getTreeDepth();
    octomap::OcTreeKey origin_key;
    if( !mOctree->coordToKeyChecked(octomap::point3d(0, 0, 0), max_depth, origin_key) )
        throw std::runtime_error("(OctomapInterface::ConvertToOccupancyGrid) Failed to convert robot origin to OcTreeKey.");
    for(octomap::ColorOcTree::leaf_iterator iter = mOctree->begin_leafs(); iter != mOctree->end_leafs(); iter++)
    {
        if( !CheckHeight(iter) )
            continue;
        
        char occVal = mOctree->isNodeOccupied(*iter) ? common::OccupancyMap2D::OccupiedCell : common::OccupancyMap2D::FreeCell;
        
        if( iter.getDepth() == max_depth )
        {
            const octomap::OcTreeKey key = iter.getKey();
            int i = key[0] - origin_key[0] + center_i;
            int j = key[1] - origin_key[1] + center_j;
            int index = map.Index(i, j);
            if( map.IsValidIndex(index) )
                map.At(i, j) = occVal; // only set it it is withing the boundary of the map.
        }
        else
        {
            // should break this node down
            int intSize = 1 << (max_depth - iter.getDepth());
            const octomap::OcTreeKey minKey = iter.getIndexKey();
            for(int dx = 0; dx < intSize; dx++)
            {
                int i = minKey[0] + dx - origin_key[0] + center_i;
                for(int dy = 0; dy < intSize; dy++)
                {
                    int j = minKey[1] + dy - origin_key[1] + center_j;
                    int index = map.Index(i, j);
                    if( map.IsValidIndex(index) )
                        map.At(index) = occVal;
                }
            }
        }
    }
    
    if( lock_interface )
        UnlockData();
}

bool OctomapInterface::CheckHeight(const OcTreeType::leaf_iterator& iter) const
{
    //TODO: Ask a question about it later
    if( mEnableHeightFiltering )
    {
        float val = iter.getCoordinate()(mHeightFilterAxis);
        return val >= mHeightFilterMinValue && val <= mHeightFilterMaxValue;
    }
    return true;
}

bool OctomapInterface::ExtractGroundPlane(const common::KinectPointCloud& pc, common::KinectPointCloud& ground, common::KinectPointCloud& nonground)
{
    ground.header = pc.header;
    nonground.header = pc.header;
    if( pc.size() < 50 )
    {
        // too small 
        return false;
    }
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    
    pcl::SACSegmentation<common::KinectPointCloud::PointType> seg;
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold(mGroundFilter.Distance);
    Eigen::Vector3f axis(0, 0, 0);
    axis[mGroundFilter.Axis] = 1;
    seg.setAxis(axis);
    seg.setEpsAngle(mGroundFilter.Angle);
    
    common::KinectPointCloud cloud_filtered(pc);
    pcl::ExtractIndices<common::KinectPointCloud::PointType> extract;
    bool bGroundPlaneFound = false;
    
    while( cloud_filtered.size() > 10 && !bGroundPlaneFound )
    {
        seg.setInputCloud(cloud_filtered.makeShared());
        seg.segment(*inliers, *coefficients);
        
        if( inliers->indices.size() == 0 )
            break; // no plane found
        
        extract.setInputCloud(cloud_filtered.makeShared());
        extract.setIndices(inliers);
        
        if( std::abs(coefficients->values.at(3)) < mGroundFilter.PlaneDistance )
        {
            extract.setNegative(false);
            extract.filter(ground);
            
            if( inliers->indices.size() != cloud_filtered.size() )
            {
                extract.setNegative(true);
                common::KinectPointCloud cloud_out;
                extract.filter(cloud_out);
                nonground += cloud_out;
                cloud_filtered = cloud_out;
            }
            bGroundPlaneFound = true;
        }
        else
        {
            // a plane was found, but not the ground plane.
            pcl::PointCloud<common::KinectPointCloud::PointType> cloud_out;
            extract.setNegative(false);
            extract.filter(cloud_out);
            nonground += cloud_out;
            
            if(inliers->indices.size() != cloud_filtered.size())
            {
                extract.setNegative(true);
                cloud_out.points.clear();
                extract.filter(cloud_out);
                cloud_filtered = cloud_out;
            }
            else
            {
                cloud_filtered.points.clear();
            }
        }
    }
    
    //TODO: SHOULD CHECK FOR STARES HERE
    
    if( !bGroundPlaneFound )
    {
        pcl::PassThrough<common::KinectPointCloud::PointType> second_pass;
        second_pass.setFilterFieldName( mGroundFilter.Axis == 0 ? "x" : (mGroundFilter.Axis == 1 ? "y" : "z") );
        
        second_pass.setFilterLimits(-mGroundFilter.PlaneDistance, mGroundFilter.PlaneDistance);
        second_pass.setInputCloud(pc.makeShared());
        second_pass.filter(ground);
        
        second_pass.setFilterLimitsNegative(true);
        second_pass.filter(nonground);
    }
    
    return true;
}


} // end namespace interfaces
} // end namespace KSRobot

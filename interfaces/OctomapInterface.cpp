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
#include <common/PCLUtils.h>

#include <sstream>
#include <pcl/io/pcd_io.h>

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
    mLazyEval(DEFAULT_LAZY_EVAL), mFilteredCloud(new common::KinectPointCloud), mGroundPC(new common::KinectPointCloud),
    mNonGroundPC(new common::KinectPointCloud)
{
    SetInterfaceName("OctomapInterface");
    mHeightFilter.Enabled = false;
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
    
//     if( mGroundFilter.Enabled && ! mLocalTransformInfo.Enabled )
//         throw std::runtime_error("(OctomapInterface::Initialize) Cannot enable ground filter while local transform filter is disabled.");
    
    if( mLocalTransformInfo.Enabled )
    {
        Eigen::AngleAxisf r1, r2;
        r1 = r2 = Eigen::AngleAxisf::Identity();
        if( mLocalTransformInfo.FirstRotationAxis != -1 )
        {
            Eigen::Vector3f axis(0, 0, 0);
            axis[mLocalTransformInfo.FirstRotationAxis] = 1;
            r1 = Eigen::AngleAxisf(mLocalTransformInfo.FirstRotationDegree * M_PI / 180, axis);
        }
        if( mLocalTransformInfo.SecondRotationAxis != -1 )
        {
            Eigen::Vector3f axis(0, 0, 0);
            axis[mLocalTransformInfo.SecondRotationAxis] = 1;
            r2 = Eigen::AngleAxisf(mLocalTransformInfo.SecondRotationDegree * M_PI / 180, axis);
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

void OctomapInterface::LoadFromFile(const std::string& filename)
{
    OcTreeType* ptr = dynamic_cast<OcTreeType*>(octomap::AbstractOcTree::read(filename));
    mOctree.reset(ptr);
}

void OctomapInterface::ReadSettings(common::ProgramOptions::Ptr po)
{
    common::Interface::ReadSettings(po);
    
    mUseColor = po->GetBool("UseColor", DEFAULT_USE_COLOR);
    mMapResolution = po->GetDouble("MapResolution", DEFAULT_MAP_RESOLUTION);
    mMaxRange = po->GetDouble("MaxRange", DEFAULT_MAX_RANGE);
    mLazyEval = po->GetBool("LazyEval", DEFAULT_LAZY_EVAL);
    mApplyVoxelGrid = po->GetBool("VoxelGrid.Enabled", DEFAULT_APPLY_VOXEL_GRID);
    mVoxelGridResolution = po->GetDouble("VoxelGrid.Resolution", DEFAULT_VOXEL_GRID_RESOLUTION);
    
    mHeightFilter.Enabled = po->GetBool("HeightFilter.Enabled", false);
    mHeightFilter.Axis = po->GetAxis("HeightFilter.Axis", std::string("z"));
    assert(mHeightFilter.Axis != -1);
    
    mHeightFilter.MaxValue = po->GetDouble("HeightFilter.MaxValue", 0.0);
    mHeightFilter.MinValue = po->GetDouble("HeightFilter.MinValue", 0.0);
    
    mGroundFilter.Enabled = po->GetBool("GroundFilter.Enabled", false);
    mGroundFilter.Distance = po->GetDouble("GroundFilter.Distance", 0.04);
    mGroundFilter.PlaneDistance = po->GetDouble("GroundFilter.PlaneDistance", 0.04);
    mGroundFilter.Angle = po->GetDouble("GroundFilter.Angle", 0.15);
    mGroundFilter.AddGroundPointsAsFreeSpace = po->GetBool("GroundFilter.AddGroundPointsAsFreeSpace", true);
    mGroundFilter.Axis = po->GetAxis("GroundFilter.Axis", std::string("z"));
    assert(mGroundFilter.Axis != -1);
    
    common::ProgramOptions::Ptr p = po->StartNode("LocalTransform");
    mLocalTransformInfo.Enabled = p->GetBool("Enabled", false);
    mLocalTransformInfo.RobotHeight = p->GetDouble("RobotHeight", 0.0);
    mLocalTransformInfo.RobotHeightAxis = p->GetAxis("RobotHeightAxis", std::string("z"));
    assert(mLocalTransformInfo.RobotHeightAxis != -1);
    
    mLocalTransformInfo.FirstRotationAxis = p->GetAxis("FirstRotationAxis", std::string("none"));
    mLocalTransformInfo.FirstRotationDegree = p->GetDouble("FirstRotationDegree", 0.0);
    mLocalTransformInfo.SecondRotationAxis = p->GetAxis("SecondRotationAxis", std::string("none"));
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

    common::KinectPointCloud pc; // a useful temp var
    
    while( mKeyframesQueue.try_pop(mi) )
    {
        mFilteredCloud->clear();
        mGroundPC->clear();
        mNonGroundPC->clear();
        
        common::Interface::ScopedLock locker(this);
        count++;
        
        if( mApplyVoxelGrid )
        {
            common::PCLUtils::ApplyVoxelGrid(*mFilteredCloud, mi.PointCloud, mVoxelGridResolution);
        }
        else
        {
            // copy this pointcloud! not the best performance, but causes better and cleaner program
            // since we usually should use voxel grids and pass throughs, then this is no problem
            *mFilteredCloud = *(mi.PointCloud);
        }

        if( mPassThrough.Enabled )
        {
            common::PCLUtils::ApplyPassThrough(pc, mFilteredCloud, "z", mPassThrough.LimitMin, mPassThrough.LimitMax);
            *mFilteredCloud = pc;
        }

        if( mLocalTransformInfo.Enabled )
        {
            pcl::transformPointCloud(*mFilteredCloud, *mFilteredCloud, mCameraTransformCached);
            //TODO: VERY IMPORTANT: I Should change mi.Transform according to mCameraTransformCached
        }
        if( mGroundFilter.Enabled && ExtractGroundPlane(*mFilteredCloud, *mGroundPC, *mNonGroundPC) )
        {
//             try
//             {
//                 std::stringstream ss1,ss2;
//                 ss1 << "ground_" << GetCycle() << ".pcd";
//                 pcl::io::savePCDFileBinary(ss1.str(), *mGroundPC);
//                 ss2 << "nonground_" << GetCycle() << ".pcd";
//                 pcl::io::savePCDFileBinary(ss2.str(), *mNonGroundPC);
//             }
//             catch(...)
//             {
//             }
            ConvertInternal(mNonGroundPC, mGroundPC, mi.Transform);
            UpdateMap(mNonGroundPC, mi.Transform.translation());
        }
        else
        {
            ConvertInternal(mFilteredCloud, common::KinectPointCloud::Ptr(), mi.Transform);
            UpdateMap(mFilteredCloud, mi.Transform.translation());
        }
        
        FinishCycle();
    }
    return count != 0;
}

void OctomapInterface::ConvertInternal(common::KinectPointCloud::ConstPtr nonground, common::KinectPointCloud::ConstPtr ground, 
                                       const Eigen::Isometry3f& transform)
{
    mOctoNonGroundPoints.clear();
    if( mGroundFilter.AddGroundPointsAsFreeSpace && ground.get() != 0 )
    {
        mOctoGroundPoints.clear();
        for(size_t i = 0; i < ground->size(); i++)
        {
            const common::KinectPointCloud::PointType& p = ground->at(i);
            if( pcl::isFinite(p) )
                mOctoGroundPoints.push_back(p.x, p.y, p.z);
        }
    }
    
    if( !mUseColor )
    {
        for(size_t i = 0; i < nonground->size(); i++)
        {
            const common::KinectPointCloud::PointType& p = nonground->at(i);
            if( pcl::isFinite(p) )
                mOctoNonGroundPoints.push_back(p.x, p.y, p.z);
        }
    }
    else
    {
        mIndexes.clear();
        mIndexes.reserve(nonground->size());
        for(size_t i = 0; i < nonground->size(); i++)
        {
            const common::KinectPointCloud::PointType& p = nonground->at(i);
            if( pcl::isFinite(p) )
            {
                mOctoNonGroundPoints.push_back(p.x, p.y, p.z);
                mIndexes.push_back(i);
            }
        }
    }
    
    const Eigen::Quaternionf eigen_rot(transform.linear());
    const Eigen::Vector3f translate = transform.translation();
    octomath::Quaternion rot(eigen_rot.w(), eigen_rot.x(), eigen_rot.y(), eigen_rot.z());
    octomath::Pose6D pose(octomath::Vector3(translate[0], translate[1], translate[2]), rot);
    mOctoNonGroundPoints.transform(pose);
    mOctoGroundPoints.transform(pose);
}

//TODO: What should I pass as compute update? 
void OctomapInterface::UpdateMap(common::KinectPointCloud::ConstPtr pc, const Eigen::Vector3f& translate)
{
    octomap::KeySet occupied, free;
    octomap::point3d sensor_origin(translate[0], translate[1], translate[2]);
    
    mOctree->computeUpdate(mOctoNonGroundPoints, sensor_origin, free, occupied, mMaxRange);
    
    for(octomap::KeySet::iterator iter = free.begin(); iter != free.end(); iter++)
        mOctree->updateNode(*iter, false, mLazyEval); // add a free cell
    
    for(octomap::KeySet::iterator iter = occupied.begin(); iter != free.end(); iter++)
        mOctree->updateNode(*iter, true, mLazyEval); // add an occupied cell
    
    if( mUseColor )
    {
        int index = 0;
        for(octomap::KeySet::iterator iter = occupied.begin(); iter != free.end(); iter++, index++)
        {
            const common::KinectPointCloud::PointType& p = pc->at(mIndexes[index]);
            mOctree->integrateNodeColor(*iter, p.r, p.g, p.b);
        }
    }
    
    // now add ground points
    if( mGroundFilter.AddGroundPointsAsFreeSpace )
    {
        free.clear();
        occupied.clear();
        mOctree->computeUpdate(mOctoGroundPoints, sensor_origin, free, occupied, mMaxRange);
        //set BOTH free and occupied cells as free. 
        //NOTE: This is beneficial for occupancy grid generation and frontier extraction
        for(octomap::KeySet::iterator iter = free.begin(); iter != free.end(); iter++)
            mOctree->updateNode(*iter, false, mLazyEval); // add a free cell
            
        for(octomap::KeySet::iterator iter = occupied.begin(); iter != free.end(); iter++)
            mOctree->updateNode(*iter, false, mLazyEval); // again, add it as a free cell. 
    }
}

//it's a helper inline function to put a value when converting map to occupancy grid
static inline void PutValueToMap(common::OccupancyMap::Ptr map, int i, int j, char val)
{
    map->AddPointToROI(i, j);
    if( map->At(i, j) != common::OccupancyMap::OccupiedCell )
        map->At(i, j) = val; // only change the value of the map if it was not occupied. (if we do not do this, there might be cases that occupied 
                             // cell be overwritten by a free cell.
}

void OctomapInterface::ConvertToOccupancyGrid(common::OccupancyMap::Ptr map, int center_i, int center_j, bool lock_interface)
{
    if( lock_interface )
        LockData();
    
    const unsigned int max_depth = mOctree->getTreeDepth(); // this should be 16 at the time of implementation.
    octomap::OcTreeKey origin_key;
    if( !mOctree->coordToKeyChecked(octomap::point3d(0, 0, 0), max_depth, origin_key) )
        throw std::runtime_error("(OctomapInterface::ConvertToOccupancyGrid) Failed to convert robot origin to OcTreeKey.");

    map->Resolution = mMapResolution;
    map->ROI.Top = std::numeric_limits<size_t>::max();
    map->ROI.Left = std::numeric_limits<size_t>::max();
    map->ROI.Width = 0;
    map->ROI.Height = 0;
    
    int axis1 = 2, axis2 = 0;
    for(octomap::ColorOcTree::leaf_iterator iter = mOctree->begin_leafs(); iter != mOctree->end_leafs(); iter++)
    {
        if( !CheckHeight(iter) )
            continue;
        
        char occVal = mOctree->isNodeOccupied(*iter) ? common::OccupancyMap::OccupiedCell : common::OccupancyMap::FreeCell;
        int numCellsMarked = 0;
        
        if( iter.getDepth() == max_depth )
        {
            numCellsMarked = 1;
            const octomap::OcTreeKey key = iter.getKey();
            int i = key[axis1] - origin_key[axis1] + center_i;
            int j = key[axis2] - origin_key[axis2] + center_j;
            
            assert(i >= 0 && j >= 0);
            if( map->IsValidPosition(i, j) )
                PutValueToMap(map, i, j, occVal);
        }
        else
        {
            // should break this node down
            int intSize = 1 << (max_depth - iter.getDepth());
            const octomap::OcTreeKey minKey = iter.getIndexKey();
            numCellsMarked = intSize * intSize; // number of cells affected
            for(int dx = 0; dx < intSize; dx++)
            {
                int i = minKey[axis1] + dx - origin_key[axis1] + center_i;
                for(int dy = 0; dy < intSize; dy++)
                {
                    int j = minKey[axis2] + dy - origin_key[axis2] + center_j;
                    assert(i >= 0 && j >= 0);
                    if( map->IsValidPosition(i, j) )
                        PutValueToMap(map, i, j, occVal);
                }
            }
        }
        
        //Now update book keeping info.
        map->UnknownCellsCount -= numCellsMarked;
        if( occVal == common::OccupancyMap::OccupiedCell )
            map->OccupiedCellsCount += numCellsMarked;
        else
            map->FreeCellsCount += numCellsMarked;
    }
    
    if( lock_interface )
        UnlockData();
}

bool OctomapInterface::CheckHeight(const OcTreeType::leaf_iterator& iter) const
{
    //TODO: Ask a question about it later
    if( mHeightFilter.Enabled )
    {
        float val = iter.getCoordinate()(mHeightFilter.Axis);
        return val >= mHeightFilter.MinValue && val <= mHeightFilter.MaxValue;
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
    
    common::KinectPointCloud filtered_cloud(pc);
    pcl::ExtractIndices<common::KinectPointCloud::PointType> extract;
    bool bGroundPlaneFound = false;
    
    while( filtered_cloud.size() > 10 && !bGroundPlaneFound )
    {
        seg.setInputCloud(filtered_cloud.makeShared());
        seg.segment(*inliers, *coefficients);
        
        if( inliers->indices.size() == 0 )
            break; // no plane found
        
        extract.setInputCloud(filtered_cloud.makeShared());
        extract.setIndices(inliers);
        
        Debug("Plane found with coefs: (%f, %f, %f, %f)\n", coefficients->values[0], coefficients->values[1], coefficients->values[2],
               coefficients->values[3]);
        
        if( std::abs(coefficients->values.at(3)) > mGroundFilter.PlaneDistance )
        {
            extract.setNegative(false);
            extract.filter(ground);
            
            if( inliers->indices.size() != filtered_cloud.size() )
            {
                extract.setNegative(true);
                common::KinectPointCloud cloud_out;
                extract.filter(cloud_out);
                nonground += cloud_out;
                filtered_cloud = cloud_out;
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
            
            if(inliers->indices.size() != filtered_cloud.size())
            {
                extract.setNegative(true);
                cloud_out.points.clear();
                extract.filter(cloud_out);
                filtered_cloud = cloud_out;
            }
            else
            {
                filtered_cloud.points.clear();
            }
        }
    }
    
    if( !bGroundPlaneFound )
    {
        //FIXME: This code does not work.
        Debug("Ground plane not found\n");
//         const std::string field_name(mGroundFilter.Axis == 0 ? "x" : (mGroundFilter.Axis == 1 ? "y" : "z"));
//         common::KinectPointCloud::Ptr pcshared = pc.makeShared();
//         common::PCLUtils::ApplyPassThrough(ground, pcshared, field_name, 
//                                            -mGroundFilter.PlaneDistance, mGroundFilter.PlaneDistance);
//         common::PCLUtils::ApplyPassThrough(nonground, pcshared, field_name,
//                                            -mGroundFilter.PlaneDistance, mGroundFilter.PlaneDistance, true);
    }
    return true;
}


} // end namespace interfaces
} // end namespace KSRobot

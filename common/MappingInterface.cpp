/*
 * <one line to give the library's name and an idea of what it does.>
 * Copyright (C) 2013  Kourosh <email>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <common/MappingInterface.h>
#include <common/KinectInterface.h>

#include <octomap/ColorOcTree.h>
#include <octomap/math/Pose6D.h>
#include <octomap/math/Quaternion.h>
#include <octomap/math/Vector3.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <vector>
#include <math.h>

#include <tbb/concurrent_queue.h>


namespace KSRobot
{
namespace common
{

class MapElement
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    MapElement() {;}
    MapElement(const MapElement& o) : PC(o.PC), Transform(o.Transform) {;}
    ~MapElement() {;}
    
    MapElement& operator = (const MapElement& o) { PC = o.PC; Transform = o.Transform; return *this; }
    
    Eigen::Isometry3f               Transform;
    KinectPointCloud::ConstPtr      PC;
};    

class MappingInterface::KOctreeMap : public octomap::ColorOcTree
{
public:
    typedef octomap::ColorOcTree                BaseType;
    typedef KOctreeMap                          ThisType;
    

    
    typedef tbb::concurrent_bounded_queue<MapElement, Eigen::aligned_allocator<MapElement> >   QueueType;
    QueueType                           mQ;
    
    
    pcl::VoxelGrid<pcl::PointXYZRGBA>           mGrid;
    KinectPointCloud::Ptr                       mFilteredCloud;
    
    octomap::Pointcloud         mPC;
    std::vector<int>            mIndexes;
    double                      mMaxRange;
    
    KOctreeMap(double res = 0.01, double max_range = 5.0) : BaseType(res), mMaxRange(max_range) {;}
    
    void Convert(common::KinectPointCloud::ConstPtr pc, const octomath::Quaternion rot)
    {
        // also update mIndexes
        mIndexes.clear();
        mIndexes.reserve(pc->size()); // some are wasted, but it's better than many reallocations
        mPC.clear();
        for(unsigned int i = 0; i < pc->points.size(); i++)
        {
            const pcl::PointXYZRGBA& p = pc->points[i];
            if( pcl::isFinite(p) )
            {
                mPC.push_back(p.x, p.y, p.z);
                mIndexes.push_back(i);
            }
        }
        
        mPC.transform(octomath::Pose6D(octomath::Vector3(0, 0, 0), rot));
    }
    
    void InsertPointCloud(common::KinectPointCloud::ConstPtr pc, const Eigen::Isometry3f& pose, bool lazy_eval = false)
    {
        Eigen::Vector3f eig_trans(pose.translation());
        Eigen::Quaternionf eig_quat(pose.linear());
        
        octomath::Vector3 trans(eig_trans[0], eig_trans[1], eig_trans[2]);
        octomath::Quaternion rot(eig_quat.w(), eig_quat.x(), eig_quat.y(), eig_quat.z());
        
        octomap::KeySet free_cells, occupied_cells;
        Convert(pc, rot);
        computeUpdate(mPC, trans, free_cells, occupied_cells, mMaxRange);
        
        for(octomap::KeySet::iterator it = free_cells.begin(); it != free_cells.end(); it++)
        {
            updateNode(*it, false, lazy_eval);
        }
        
        int index = 0;
        for(octomap::KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); it++, index++)
        {
            const pcl::PointXYZRGBA& p = pc->points[mIndexes[index]];
            updateNode(*it, true, lazy_eval);
            integrateNodeColor(*it, p.r, p.g, p.b);
        }
    }
    
    void Test()
    {
        for(leaf_iterator iter = this->begin_leafs(); iter != this->end_leafs(); iter++)
        {
            //TODO: 
        }
    }

};

// MappingInterface::MappingInterface(): Interface(""), mMapRes(0.01), mMaxRange(-1),
// mApplyFilter(true), mFilterTimer(new Timer("FilterDownSample")), mUpdateTimer(new Timer("Update"))
// {
//     mQ.set_capacity(1000); // Storing this much pointcloud will need 5GB of memory, so it's better to fail!
//     ReInitialize();
//     
//     RegisterTimer(mFilterTimer);
//     RegisterTimer(mUpdateTimer);
// }


MappingInterface::MappingInterface() : Interface(), mMapRes(0.01), mMaxRange(-1),
    mApplyFilter(true), mFilterTimer(new Timer("FilterDownSample")), mUpdateTimer(new Timer("Update"))
{
    ReInitialize();
    
    RegisterTimer(mFilterTimer);
    RegisterTimer(mUpdateTimer);
}

MappingInterface::~MappingInterface()
{
}

void MappingInterface::ReInitialize()
{
    mMapper.reset(new KOctreeMap(mMapRes, mMaxRange));
    mMapper->mQ.set_capacity(1000); // Storing this much pointcloud will need 5GB of memory, so it's better to fail!
    mMapper->mGrid.setLeafSize(mMapRes / 2, mMapRes / 2, mMapRes / 2);
    mMapper->mFilteredCloud.reset(new KinectPointCloud());
}

void MappingInterface::OnNewKeypoint()
{
    if( !mVO->IsThisCycleKeyframe() )
        return;
    
    MapElement me;
    me.PC = mVO->GetCurrentPointCloud();
    me.Transform = mVO->GetGlobalPose();
    mMapper->mQ.try_push(me);
}

void MappingInterface::RegisterToVO(VisualOdometryInterface::Ptr vo)
{
    mVO = vo;
    mVO->RegisterKeyframeReceiver(boost::bind(&MappingInterface::OnNewKeypoint, this));
}

bool MappingInterface::RunSingleCycle()
{
    int count = 0;
    MapElement me;
    while( mMapper->mQ.try_pop(me) )
    {
        KinectPointCloud::ConstPtr final_output;
        if( mApplyFilter )
        {
            mFilterTimer->Start();
                mMapper->mFilteredCloud->points.clear(); // in case the filter doesn't do this
                mMapper->mGrid.setInputCloud(me.PC);
                mMapper->mGrid.filter(*(mMapper->mFilteredCloud));
                
                Eigen::Quaternionf quat(Eigen::AngleAxisf(6, Eigen::Vector3f::UnitX()));
                pcl::transformPointCloud(*(mMapper->mFilteredCloud), *(mMapper->mFilteredCloud), Eigen::Vector3f(0,0,0), quat);
                
                final_output = mMapper->mFilteredCloud;
            mFilterTimer->Stop();
        }
        else
            final_output = me.PC;
        
        
        mUpdateTimer->Start();
            mMapper->InsertPointCloud(final_output, me.Transform);
        mUpdateTimer->Stop();
        count++;
    }
    
    if( count != 0 )
        FinishCycle();
    
    return (count != 0);
}

void MappingInterface::SaveMapToFile(const std::string& filename)
{
    mMapper->write(filename);
}

void MappingInterface::ReadSettings(ProgramOptions::Ptr po)
{
    Interface::ReadSettings(po);
    
    mMapRes = po->GetDouble("MapResolution", 0.1);
    mMaxRange = po->GetDouble("MaxRange", -1.0);
    mApplyFilter = po->GetBool("ApplyFilter", true);
    
    ReInitialize();
}

} // end namespace common
} // end namespace KSRobot

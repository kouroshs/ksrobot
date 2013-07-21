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

#ifndef OCTOMAPINTERFACE_H
#define OCTOMAPINTERFACE_H

#include <common/MappingInterface.h>
#include <common/KinectInterface.h>

#include <pcl/filters/voxel_grid.h>
#include <octomap/AbstractOccupancyOcTree.h>
#include <tbb/concurrent_queue.h>
#include <sys/mman.h>

namespace KSRobot
{
namespace interfaces
{

class OctomapInterface : public common::MappingInterface
{
public:
    typedef OctomapInterface                    this_type;
    typedef boost::shared_ptr<this_type>        Ptr;
    typedef boost::shared_ptr<const this_type>  ConstPtr;
    
    OctomapInterface();
    virtual ~OctomapInterface();
    
    virtual void Initialize();
    
    inline boost::shared_ptr<octomap::AbstractOccupancyOcTree>      GetOctree() const;
    inline bool                                                     GetUseColor() const;
    inline void                                                     SetUseColor(bool useColor);
    inline float                                                    GetMapResolution() const;
    inline void                                                     SetMapResolution(float res);
    inline float                                                    GetMaxRange() const;
    inline void                                                     SetMaxRange(float range);
    inline bool                                                     GetApplyVoxelGrid() const;
    inline void                                                     SetApplyVoxelGrid(bool enable);
    inline float                                                    GetVoxelGridResolution() const;
    inline void                                                     SetVoxelGridResolution(float res);
    
protected:
    boost::shared_ptr<octomap::AbstractOccupancyOcTree>     mOctree;
    
    bool                                                    mUseColor;
    float                                                   mMapResolution;
    float                                                   mMaxRange;
    
    bool                                                    mApplyVoxelGrid;
    float                                                   mVoxelGridResolution;
    
    common::KinectPointCloud::Ptr                           mFilteredCloud;
    pcl::VoxelGrid<common::KinectPointCloud::PointType>     mVoxelGrid;
};

inline boost::shared_ptr<octomap::AbstractOccupancyOcTree> OctomapInterface::GetOctree() const
{
    return mOctree;
}

bool OctomapInterface::GetUseColor() const
{
    return mUseColor;
}

void OctomapInterface::SetUseColor(bool useColor)
{
    mUseColor = useColor;
}

float OctomapInterface::GetMapResolution() const
{
    return mMapResolution;
}

void OctomapInterface::SetMapResolution(float res)
{
    mMapResolution = res;
}

float OctomapInterface::GetMaxRange() const
{
    return mMaxRange;
}

void OctomapInterface::SetMaxRange(float range)
{
    mMaxRange = range;
}

bool OctomapInterface::GetApplyVoxelGrid() const
{
    return mApplyVoxelGrid;
}

void OctomapInterface::SetApplyVoxelGrid(bool enable)
{
    mApplyVoxelGrid = enable;
}

float OctomapInterface::GetVoxelGridResolution() const
{
    return mVoxelGridResolution;
}

void OctomapInterface::SetVoxelGridResolution(float res)
{
    mVoxelGridResolution = res;
}



} // end namespace interfaces
} // end namespace KSRobot
    

#endif // OCTOMAPINTERFACE_H

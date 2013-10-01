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

#ifndef MAPPINGINTERFACE_H
#define MAPPINGINTERFACE_H

#include <common/Interface.h>
#include <common/VisualOdometryInterface.h>
#include <common/OccupancyMap.h>
#include <common/Timer.h>
#include <Eigen/Geometry>

#include <tbb/concurrent_queue.h>

namespace KSRobot
{
namespace common
{

class MappingInterface : public Interface
{
public:
    typedef MappingInterface                    this_type;
    typedef boost::shared_ptr<this_type>        Ptr;
    typedef boost::shared_ptr<const this_type>  ConstPtr;
    
    MappingInterface();
    virtual ~MappingInterface();
    
    // Not neccessary to call
    virtual void                                Initialize();
    void                                        RegisterToVO(VisualOdometryInterface::Ptr vo);
    virtual void                                SaveToFile(const std::string& filename) = 0;
    virtual void                                LoadFromFile(const std::string& filename) = 0; // for  debugging only
    virtual void                                SetRobotInfo(RobotInfo::Ptr roboinfo); // TODO: IMPLEMENT
    
    virtual void                                ConvertToOccupancyGrid(OccupancyMap::Ptr map, int center_i, int center_j, 
                                                                       bool lock_interface = false) = 0;
protected:
    void                                        OnNewKeyframe();
protected:
    VisualOdometryInterface::Ptr                mVO;
    struct MapInfo
    {
        Eigen::Isometry3f                       Transform;
        common::KinectPointCloud::ConstPtr      PointCloud;
    };
    
    tbb::concurrent_bounded_queue<MapInfo>      mKeyframesQueue;
    bool                                        mHeightSet;
    float                                       mRobotHeight;
    float                                       mRobotRadius;
    
    CLASS_DEF_PYEXPORT;
};

} // end namespace common
} // end namespace KSRobot

#endif // MAPPINGINTERFACE_H

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
#include <common/Timer.h>
#include <Eigen/Geometry>

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
    inline double                       GetMapResolution() const;
    inline double                       GetMaxRange() const;
    inline void                         SetMapResolution(const double& res);
    inline void                         SetMaxRange(const double& range);
    inline bool                         IsFilteringEnabled() const;
    inline void                         EnableFiltering(bool enable);
    
    void                                ReInitialize();
    void                                RegisterToVO(VisualOdometryInterface::Ptr vo);
    
    void                                SaveMapToFile(const std::string& filename);
    
    virtual void                        ReadSettings(ProgramOptions::Ptr po);
    
    virtual bool                        RunSingleCycle();
private:
    void                                OnNewKeypoint();
private:
    //TODO: Add list of kinect pointclouds
    VisualOdometryInterface::Ptr        mVO;
    class KOctreeMap;
    boost::shared_ptr<KOctreeMap>       mMapper;
    
    double                              mMapRes;
    double                              mMaxRange;
    bool                                mApplyFilter;
    
    Timer::Ptr                          mFilterTimer;
    Timer::Ptr                          mUpdateTimer;
};

inline double MappingInterface::GetMapResolution() const
{
    return mMapRes;
}

inline double MappingInterface::GetMaxRange() const
{
    return mMaxRange;
}

inline void MappingInterface::SetMapResolution(const double& res)
{
    mMapRes = res;
}

inline void MappingInterface::SetMaxRange(const double& range)
{
    mMaxRange = range;
}

inline void MappingInterface::EnableFiltering(bool enable)
{
    mApplyFilter = enable;
}

inline bool MappingInterface::IsFilteringEnabled() const
{
    return mApplyFilter;
}



} // end namespace common
} // end namespace KSRobot

#endif // MAPPINGINTERFACE_H

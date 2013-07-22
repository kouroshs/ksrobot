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

#include <tbb/concurrent_queue.h>

namespace KSRobot
{
namespace common
{

class OccupancyMap2D
{
public:
    
    enum CellValue
    {
        OccupiedCell    = 100,
        FreeCell        = 0,
        UnknownCell     = 50
    };
    
    OccupancyMap2D(int width, int height) : mWidth(width), mHeight(height), mData(width * height, UnknownCell) {;}
    ~OccupancyMap2D();
    
    int                     GetWidth() const { return mWidth; }
    void                    SetWidth(int w) { mWidth = w; }
    int                     GetHeight() const { return mHeight; }
    void                    SetHeight(int h) { mHeight = h; }
    int                     GetSize() const { return mWidth * mHeight; }
    
    int                     Index(int i, int j) const { return mWidth * i + j; }
    
    char                    At(int idx) const { return mData[idx]; }
    char&                   At(int idx) { return mData[idx]; }
    
    char                    At(int i, int j) const { return mData[Index(i, j)]; }
    char&                   At(int i, int j) { return mData[Index(i, j)]; }
    
    bool                    IsValidIndex(int idx) const { return idx < (int)mData.size(); }
    
private:
    int                     mWidth;
    int                     mHeight;
    std::vector<char>       mData;
};

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
    
    virtual void                                ConvertToOccupancyGrid(OccupancyMap2D& map, int center_i, int center_j, bool lock_interface = false) = 0;
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
};

} // end namespace common
} // end namespace KSRobot

#endif // MAPPINGINTERFACE_H

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
    typedef unsigned char           MapPointDataType;
    
    struct Frontier
    {
        //size_t              Position;
        //size_t              Population;
        float               X, Y;
        float               Rotation;
        size_t              Size;
        MapPointDataType    ID;
    };
    
    class Rect
    {
    public:
        Rect() : Top(0), Left(0), Width(0), Height(0) {;}
        
        size_t      Top;
        size_t      Left;
        size_t      Width;
        size_t      Height;
        
        size_t      Right() const { return Left + Width; }
        size_t      Buttom() const { return Top + Height; }
    };
    
    enum CellValue
    {
        UnknownCell     = 0,
        OccupiedCell    = 1,
        FreeCell        = 2,
        FrontierCell    = 3,
        
        FrontierStartID = 4,
        FrontierEndID = 255
    };
    
    OccupancyMap2D(size_t width, size_t height) : Data(width * height, UnknownCell), Width(width), Height(height), 
                                            OccupiedCellsCount(0), FreeCellsCount(0), UnknownCellsCount(width * height),
                                            CenterX(0), CenterY(0), Resolution(0.0f) {;}
    
    size_t                  GetWidth() const { return Width; }
    void                    SetWidth(size_t w) { Width = w; }
    size_t                  GetHeight() const { return Height; }
    void                    SetHeight(size_t h) { Height = h; }
    size_t                  GetSize() const { return Data.size(); }
    
    size_t                  Index(size_t i, size_t j) const { return Width * i + j; }
    
    MapPointDataType        At(size_t idx) const { return Data[idx]; }
    MapPointDataType&       At(size_t idx) { return Data[idx]; }
    
    MapPointDataType        At(size_t i, size_t j) const { return Data[Index(i, j)]; }
    MapPointDataType&       At(size_t i, size_t j) { return Data[Index(i, j)]; }
    
    void                    Reinitialize(); // This sets the ROI to UnknownCell
    
    bool                    IsValidIndex(size_t idx) const { return idx < Data.size(); }
    
    // This is for when an occupied or free point is added to the map.
    // Region of Interest (ROI) is for optimization, it is used to show
    // which regions are map are set so it won't be necessary to search the whole map
    // when only a small area has data.
    void                    AddPointToRIO(size_t i, size_t j)
    {
        ROI.Top = std::min(i, ROI.Top);
        ROI.Left = std::min(j, ROI.Left);
        if( i > ROI.Top + ROI.Height )
            ROI.Height = i - ROI.Top;
        if( j > ROI.Left + ROI.Width )
            ROI.Width = j - ROI.Left;
    }
    
    //NOTE: This will change the map. After this function call, every point that is in a frontier region will be assigned an ID
    //      of that frontier. Frontier ID's are in the range of FrontierStartID to FrontierEndID. So for unsigned char data type
    //      this map can support up to 252 frontiers. If more frontiers are needed then eighter we should change the data type, or
    //      change the algorithm so that the OccupancyMap2D::Frontier would take a list of frontier points. In the latter case there
    //      should be some changes to the algorithm implemented in this function to account for frontier point values not changing anymore.
    void                    ExtractFrontiers(std::vector<OccupancyMap2D::Frontier>& frontiers, float robot_diameter = 0.0f);
    
public:
    std::vector<MapPointDataType>       Data;
    size_t                              Width;
    size_t                              Height;
    // These three should be updated by mapping interface
    size_t                              OccupiedCellsCount;
    size_t                              FreeCellsCount;
    size_t                              UnknownCellsCount;
    
    size_t                              CenterX;
    size_t                              CenterY;
    float                               Resolution;
    
    Rect                                ROI;
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

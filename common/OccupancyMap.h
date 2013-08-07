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

#ifndef OCCUPANCYMAP_H
#define OCCUPANCYMAP_H

#include <Eigen/Geometry>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace KSRobot
{
namespace common
{

class OccupancyMap
{
public:
    typedef unsigned char                           MapPointDataType;
    
    typedef OccupancyMap                            this_type;
    typedef boost::shared_ptr<this_type>            Ptr;
    typedef boost::shared_ptr<const this_type>      ConstPtr;
    
    struct Frontier
    {
        //float               X, Y;
        Eigen::Vector2f     Position;
        Eigen::Matrix2f     Variance;     // Standard deviation in position TODO: IMPLEMENT THIS
        float               Rotation;
        size_t              Size;
        MapPointDataType    ID;
        
        bool operator == (const Frontier& other) { return ID == other.ID; }
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
    
    OccupancyMap() {;} // completely uninitialized
    OccupancyMap(size_t width, size_t height) : Data(width * height, UnknownCell), Width(width), Height(height), 
                                            OccupiedCellsCount(0), FreeCellsCount(0), UnknownCellsCount(width * height),
                                            CenterX(0), CenterY(0), Resolution(0.0f) {;}
    
    
    void                    LoadFromFile(const std::string& filename);
    void                    SaveToFile(const std::string& filename);
    
    size_t                  GetWidth() const { return Width; }
    void                    SetWidth(size_t w) { Width = w; }
    size_t                  GetHeight() const { return Height; }
    void                    SetHeight(size_t h) { Height = h; }
    size_t                  GetSize() const { return Data.size(); }
    
    size_t                  Index(size_t i, size_t j) const { assert(IsValidPosition(i, j)); return Width * i + j; }
    
    MapPointDataType        At(size_t idx) const { assert(IsValidIndex(idx)); return Data[idx]; }
    MapPointDataType&       At(size_t idx) { assert(IsValidIndex(idx)); return Data[idx]; }
    
    MapPointDataType        At(size_t i, size_t j) const { return Data[Index(i, j)]; }
    MapPointDataType&       At(size_t i, size_t j) { return Data[Index(i, j)]; }
    
    void                    Reinitialize(); // This sets the ROI to UnknownCell
    
    bool                    IsValidIndex(size_t idx) const { return idx < Data.size(); }
    bool                    IsValidPosition(size_t i, size_t j) const { return (i < Height && j < Width);}
    
    // This is for when an occupied or free point is added to the map.
    // Region of Interest (ROI) is for optimization, it is used to show
    // which regions are map are set so it won't be necessary to search the whole map
    // when only a small area has data.
    void                    AddPointToROI(size_t i, size_t j)
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
    void                    ExtractFrontiers(std::vector<OccupancyMap::Frontier>& frontiers, float robot_diameter = 0.0f);
    
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

} // end namespace common
} // end namespace KSRobot

#endif // OCCUPANCYMAP_H

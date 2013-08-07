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

#include <common/OccupancyMap.h>
#include <vector>
#include <math.h>
#include <fstream>
#include <stdexcept>

namespace KSRobot
{
namespace common
{

void OccupancyMap::Reinitialize()
{
    //NOTE This is only if ROI is valid.
    //TODO: Check if ROI is valid.
    for(size_t i = ROI.Top; i < ROI.Top + ROI.Height; i++)
    {
        size_t baseIdx = Index(i, ROI.Left);
        for(size_t j = 0; j < ROI.Width; j++)
            Data[baseIdx + j] = OccupancyMap::UnknownCell;
    }
}

struct FrontierPoint
{
    size_t              Index;
    Eigen::Vector2f     Direction;
    
    FrontierPoint(size_t idx, const Eigen::Vector2f& dir) : Index(idx), Direction(dir) {;}
};

void OccupancyMap::ExtractFrontiers(std::vector<OccupancyMap::Frontier>& frontiers, float robot_diameter)
{
    frontiers.clear();
    //First mark all points that are frontier.
    // Frontiers are Unoccupied cell that have unknown neighbours.
    int fc = 0;
    for(size_t i = ROI.Top; i < ROI.Buttom(); i++)
    {
        //NOTE: Could increase performance by using linear index; but the code here is more clear.
        for(size_t j = ROI.Left; j < ROI.Right(); j++)
        {
            if( At(i, j) == OccupancyMap::FreeCell && 
                ((i + 1 < Height && At(i + 1, j) == OccupancyMap::UnknownCell) ||
                 (i > 0 && At(i - 1, j) == OccupancyMap::UnknownCell) ||
                 (j + 1 < Width && At(i, j + 1) == OccupancyMap::UnknownCell) ||
                 (j > 0 && At(i, j - 1) == OccupancyMap::UnknownCell)))
            {
                //This is a frontier
                At(i, j) = OccupancyMap::FrontierCell;
                fc++;
            }
        }
    }
    
    std::vector< std::vector<FrontierPoint> >  segments;
    std::vector<size_t> neighbours;
    std::vector<FrontierPoint> seg;
    const size_t size = Data.size();
    
    MapPointDataType currFrontierID = (MapPointDataType)FrontierStartID;
    bool bContinueTopLoop = true;
    
    for(size_t row = ROI.Top; row < ROI.Buttom(); row++)
    {
        if( !bContinueTopLoop )
            break;
        size_t baseIndex = Index(row, ROI.Left);
        for(size_t col = 0; col < ROI.Width; col++)
        {
            size_t currIdx = baseIndex + col;
            
            if( Data[currIdx] != OccupancyMap::FrontierCell )
                continue;
            
            neighbours.clear();
            neighbours.push_back(currIdx);
            seg.clear();
            
            while( neighbours.size() > 0 )
            {
                size_t idx = neighbours.back();
                neighbours.pop_back();
                Data[idx] = currFrontierID;
                
                Eigen::Vector2f total(0, 0);
                int count = 0;
                
                if( idx + 1 < size && Data[idx + 1] == OccupancyMap::UnknownCell )
                {
                    count++;
                    total += Eigen::Vector2f(1.0f, 0.0f);
                }
                if( idx > 0 && Data[idx - 1] == OccupancyMap::UnknownCell )
                {
                    count++;
                    total += Eigen::Vector2f(-1.0f, 0.0f);
                }
                if( idx + Width < size && Data[idx + Width] == OccupancyMap::UnknownCell )
                {
                    count++;
                    total += Eigen::Vector2f(0.0f, 1.0f);
                }
                if( idx > Width && Data[idx - Width] == OccupancyMap::UnknownCell )
                {
                    count++;
                    total += Eigen::Vector2f(0.0f, -1.0f);
                }
                
                assert(count > 0);
                seg.push_back(FrontierPoint(idx, total / count));
                
                //Now on 8 neighbour, if any neighbour is unknown cell, push it to neighbours
                //TODO: Add code to not add borders to neighbours
                // original 4-neighbours
                if( idx > 1 && Data[idx - 1] == OccupancyMap::FrontierCell )
                    neighbours.push_back(idx - 1);
                if( idx + 1 < size  && Data[idx + 1] == OccupancyMap::FrontierCell )
                    neighbours.push_back(idx + 1);
                if( idx > Width && Data[idx - Width] == OccupancyMap::FrontierCell )
                    neighbours.push_back(idx - Width);
                if( idx + Width < size && Data[idx + Width] == OccupancyMap::FrontierCell )
                    neighbours.push_back(idx + Width);
                // now the diagonal neighbours
                if( idx > Width + 1 && Data[idx - Width - 1] == OccupancyMap::FrontierCell )
                    neighbours.push_back(idx - Width - 1);
                if( idx > Width - 1 && Data[idx - Width + 1] == OccupancyMap::FrontierCell )
                    neighbours.push_back(idx - Width + 1);
                if( idx + Width + 1 < size && Data[idx + Width + 1] == OccupancyMap::FrontierCell )
                    neighbours.push_back(idx + Width + 1);
                if( idx + Width - 1 < size && Data[idx + Width - 1] == OccupancyMap::FrontierCell )
                    neighbours.push_back(idx + Width - 1);
            }
            
            segments.push_back(seg);
            
            currFrontierID++;
            if( currFrontierID == FrontierEndID )
            {
                bContinueTopLoop = false; // cannot support any more frontiers, stop.
                break;
            }
        }
    }
    
    // now that we have all the segments, extract frontiers.
    
    for(size_t i = 0; i < segments.size(); i++)
    {
        OccupancyMap::Frontier f;
        std::vector<FrontierPoint>& curr_seg = segments[i];
        
        if( curr_seg.size() * Resolution < robot_diameter )
            continue; // the region is not possibly big enough to fit the robot.
            
        Eigen::Vector2f pos(0, 0);
        Eigen::Vector2f direction(0, 0);
        Eigen::Matrix2f xixi = Eigen::Matrix2f::Zero();
        
        for(size_t j = 0; j < curr_seg.size(); j++)
        {
            direction += curr_seg[j].Direction;
            size_t idx = curr_seg[j].Index;
            
            pos[0] += idx % Width;
            pos[1] += idx / Width;
            
            xixi += pos * pos.transpose();
        }
        
        Eigen::Vector2f sigmaPos = pos;
        pos /= curr_seg.size(); // it's average pos
        direction = direction / curr_seg.size();
        
        f.Position = pos;
        f.Variance = (pos * pos.transpose() + xixi - pos * sigmaPos.transpose() - sigmaPos * pos.transpose()) / (curr_seg.size() - 1);
        f.Rotation = atan2(direction[1], direction[0]);
        f.Size = curr_seg.size();
        f.ID = (MapPointDataType)(i + OccupancyMap::FrontierStartID); // value of the points that are in this part.
        frontiers.push_back(f);
    }
}

void OccupancyMap::LoadFromFile(const std::string& filename)
{
    std::ifstream file(filename.c_str(), std::ios::in | std::ios::binary);
    if( !file.is_open() )
        throw std::runtime_error("(OccupancyMap::ReadFromFile) Could not open file " + filename + ".");
    
    file >> Width >> Height >> OccupiedCellsCount >> FreeCellsCount >> UnknownCellsCount;
    file >> CenterX >> CenterY >> Resolution;
    file >> ROI.Top >> ROI.Left >> ROI.Width >> ROI.Height;
    
    size_t sz = Width * Height;
    Data.clear();
    Data.resize(sz);
    
    file.read((char*)Data.data(), sz * sizeof(OccupancyMap::MapPointDataType));
}

void OccupancyMap::SaveToFile(const std::string& filename)
{
    std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary);
    if( !file.is_open() )
        throw std::runtime_error("(OccupancyMap::WriteToFile) Could not open file " + filename + ".");
    
    //Order of writing:
    //TODO: Add a header for later
    //1 - Width
    //2 - Height
    //3 - OccupiedCellCount
    //4 - FreeCellsCount
    //5 - UnknownCellCount
    //6 - CenterX
    //7 - CenterY
    //8 - Resolution
    //9 - ROI { Top, Left, Width, Height }
    //10 - Data
    
    file << Width << Height << OccupiedCellsCount << FreeCellsCount << UnknownCellsCount;
    file << CenterX << CenterY << Resolution;
    file << ROI.Top << ROI.Left << ROI.Width << ROI.Height;
    file.write((char*)Data.data(), Data.size() * sizeof(OccupancyMap::MapPointDataType));
    
    file.close();
}


} // end namespace common
} // end namespace KSRobot

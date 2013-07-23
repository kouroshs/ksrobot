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

void OccupancyMap2D::Reinitialize()
{
    //NOTE This is only if ROI is valid.
    //TODO: Check if ROI is valid.
    for(size_t i = ROI.Top; i < ROI.Top + ROI.Height; i++)
    {
        size_t baseIdx = Index(i, 0);
        for(size_t j = 0; j < ROI.Width; j++)
            Data[baseIdx + j] = OccupancyMap2D::UnknownCell;
    }
}

struct FrontierPoint
{
    size_t              Index;
    Eigen::Vector2f     Direction;
    
    FrontierPoint(size_t idx, const Eigen::Vector2f& dir) : Index(idx), Direction(dir) {;}
};

void OccupancyMap2D::ExtractFrontiers(std::vector< OccupancyMap2D::Frontier >& frontiers, float robot_diameter)
{
    frontiers.clear();
    //First mark all points that are frontier.
    // Frontiers are Unoccupied cell that have unknown neighbours.
    for(size_t i = ROI.Top; i < ROI.Buttom(); i++)
    {
        //NOTE: Could increase performance by using linear index; but the code here is more clear.
        for(size_t j = ROI.Left; j < ROI.Right(); j++)
        {
            if( At(i, j) == OccupancyMap2D::FreeCell && 
                ((i + 1 < Width && At(i + 1, j) == OccupancyMap2D::UnknownCell) ||
                 (i > 0 && At(i - 1, j) == OccupancyMap2D::UnknownCell) ||
                 (j + 1 < Height && At(i, j + 1) == OccupancyMap2D::UnknownCell) ||
                 (j > 0 && At(i, j - 1) == OccupancyMap2D::UnknownCell)))
            {
                //This is a frontier
                At(i, j) = OccupancyMap2D::FrontierCell;
            }
        }
    }
    
    std::vector< std::vector<FrontierPoint> >  segments;
    std::vector<size_t> neighbours;
    std::vector<FrontierPoint> seg;
    const size_t size = Data.size();
    
    const MapPointDataType maxFrontiers = (MapPointDataType)(FrontierEndID - FrontierStartID);
    MapPointDataType currFrontierID = (MapPointDataType)FrontierStartID;
    bool bContinueTopLoop = true;
    for(size_t row = ROI.Top; row < ROI.Buttom(); row++)
    {
        if( !bContinueTopLoop )
            break;
        size_t baseIndex = Index(row, 0);
        for(size_t col = 0; col < ROI.Width; col++)
        {
            size_t currIdx = baseIndex + col;
            
            if( Data[currIdx] != OccupancyMap2D::FrontierCell )
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
                
                if( idx + 1 < size && Data[idx + 1] == OccupancyMap2D::UnknownCell )
                {
                    count++;
                    total += Eigen::Vector2f(1.0f, 0.0f);
                }
                if( idx > 0 && Data[idx - 1] == OccupancyMap2D::UnknownCell )
                {
                    count++;
                    total += Eigen::Vector2f(-1.0f, 0.0f);
                }
                if( idx + Width < size && Data[idx + Width] == OccupancyMap2D::UnknownCell )
                {
                    count++;
                    total += Eigen::Vector2f(0.0f, 1.0f);
                }
                if( idx > Width && Data[idx - Width] == OccupancyMap2D::UnknownCell )
                {
                    count++;
                    total += Eigen::Vector2f(0.0f, -1.0f);
                }
                
                assert(count > 0);
                seg.push_back(FrontierPoint(idx, total / count));
                
                //Now on 8 neighbour, if any neighbour is unknown cell, push it to neighbours
                //TODO: Add code to not add borders to neighbours
                // original 4-neighbours
                if( idx > 1 && Data[idx - 1] == OccupancyMap2D::FrontierCell )
                    neighbours.push_back(idx - 1);
                if( idx + 1 < size  && Data[idx + 1] == OccupancyMap2D::FrontierCell )
                    neighbours.push_back(idx + 1);
                if( idx > Width && Data[idx - Width] == OccupancyMap2D::FrontierCell )
                    neighbours.push_back(idx - Width);
                if( idx + Width < size && Data[idx + Width] == OccupancyMap2D::FrontierCell )
                    neighbours.push_back(idx + Width);
                // now the diagonal neighbours
                if( idx > Width + 1 && Data[idx - Width - 1] == OccupancyMap2D::FrontierCell )
                    neighbours.push_back(idx - Width - 1);
                if( idx > Width - 1 && Data[idx - Width + 1] == OccupancyMap2D::FrontierCell )
                    neighbours.push_back(idx - Width + 1);
                if( idx + Width + 1 < size && Data[idx + Width + 1] == OccupancyMap2D::FrontierCell )
                    neighbours.push_back(idx + Width + 1);
                if( idx + Width - 1 < size && Data[idx + Width - 1] == OccupancyMap2D::FrontierCell )
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
        OccupancyMap2D::Frontier f;
        std::vector<FrontierPoint>& curr_seg = segments[i];
        
        if( curr_seg.size() * Resolution < robot_diameter )
            continue; // the region is not possibly big enough to fit the robot.
            
        float x = 0, y = 0;
        Eigen::Vector2f direction(0, 0);
        
        for(size_t j = 0; j < curr_seg.size(); j++)
        {
            direction += curr_seg[j].Direction;
            size_t idx = curr_seg[j].Index;
            
            x += idx % Width;
            y += idx / Width;
        }
        direction = direction / curr_seg.size();
        
        f.X = x;
        f.Y = y;
        f.Rotation = atan2(direction[1], direction[0]);
        f.Size = curr_seg.size();
        f.ID = (MapPointDataType)(i + OccupancyMap2D::FrontierStartID); // value of the points that are in this part.
        frontiers.push_back(f);
    }
}

MappingInterface::MappingInterface() : Interface()
{
    mKeyframesQueue.set_capacity(100);
}

MappingInterface::~MappingInterface()
{
}

void MappingInterface::Initialize()
{
}

void MappingInterface::OnNewKeyframe()
{
    MapInfo mi;
    mi.PointCloud = mVO->GetCurrentPointCloud();
    mi.Transform = mVO->GetGlobalPose();
    if( !mKeyframesQueue.try_push(mi) )
        throw std::runtime_error("(MappingInterface::OnNewKeypoint) Keyframe queue is full.");
}

void MappingInterface::RegisterToVO(VisualOdometryInterface::Ptr vo)
{
    mVO = vo;
    mVO->RegisterKeyframeReceiver(boost::bind(&MappingInterface::OnNewKeyframe, this));
}

} // end namespace common
} // end namespace KSRobot

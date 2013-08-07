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

MappingInterface::MappingInterface() : Interface(), mHeightSet(false), mRobotHeight(0), mRobotRadius(0)
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

void MappingInterface::SetRobotInfo(RobotInfo::Ptr roboinfo)
{
    mHeightSet = roboinfo->AddHeightToOdometry();
    mRobotHeight = roboinfo->GetConstantHeight();
    mRobotRadius = roboinfo->GetRadius();
}


} // end namespace common
} // end namespace KSRobot

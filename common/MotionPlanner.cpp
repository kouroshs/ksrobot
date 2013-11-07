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

#include <common/MotionPlanner.h>

namespace KSRobot
{
namespace common
{

MotionPlanner::MotionPlanner()
{
    mRobotRadius = 0.0f;
    mSafeThr = 0.0f;
    mTimeout = 1.0f; // 1 second
    mGloalState.Position = mStartState.Position = Eigen::Vector2f(0, 0);
    mGloalState.Yaw = mStartState.Yaw = 0.0f;
}

MotionPlanner::~MotionPlanner()
{

}

void MotionPlanner::ReadSettings(ProgramOptions::Ptr po)
{
    mRobotRadius = po->GetDouble("RobotRadius", 0.0f);
    mSafeThr = po->GetDouble("SafeThreshold", 0.0f);
    mTimeout = po->GetDouble("Timeout", 1.0f);
}

void MotionPlanner::Initialize(size_t max_map_size)
{
    mEffectiveRadius = mRobotRadius + mSafeThr;
    mMaxMapSize = max_map_size;
}

std::string MotionPlanner::GetPlannerResultString(MotionPlanner::PlannerResult res)
{
    switch(res)
    {
        case SUCCESS:
            return "SUCCESS";
            break;
        case TIMEOUT:
            return "TIMEOUT";
            break;
        case FAIL:
            return "FAIL";
            break;
    }
    return "ILLEGAL STATE (BUG)";
}


} // end namespace common
} // end namespace KSRobot

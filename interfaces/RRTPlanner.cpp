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

#include <interfaces/RRTPlanner.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/ScopedState.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace KSRobot
{
namespace interfaces
{
//TODO: Move this to common
RRTPlanner::RRTPlanner() : common::MotionPlanner()
{
}

RRTPlanner::~RRTPlanner()
{
}

void RRTPlanner::ReadSettings(common::ProgramOptions::Ptr po)
{
    common::MotionPlanner::ReadSettings(po);
}

void RRTPlanner::Initialize()
{
    common::MotionPlanner::Initialize();
    mSpace.reset(new ob::SE2StateSpace());
    
    mSI.reset(new ob::SpaceInformation(mSpace));
    mSI->setStateValidityChecker(boost::bind(&RRTPlanner::IsValidState, this, _1));
    
    mProbDef.reset(new ob::ProblemDefinition(mSI));
    mPlanner.reset(new og::RRTConnect(mSI));
    mPlanner->setProblemDefinition(mProbDef);
    mPlanner->setup();
}

common::MotionPlanner::PlannerResult RRTPlanner::Plan()
{
    mPlan.clear();
    //TODO: Set bounds
    
    ob::ScopedState<ob::SE2StateSpace> start(mSpace);
    ob::ScopedState<ob::SE2StateSpace> goal(mSpace);
    
    start->setXY(mStartState.Position[0], mStartState.Position[1]);
    start->setYaw(mStartState.Yaw);
    goal->setXY(mGloalState.Position[0], mGloalState.Position[1]);
    goal->setYaw(mGloalState.Yaw);
    
    // extract bounds from ROI (region of interest)
    const common::OccupancyMap::Rect& roi = mOccupancyMap->ROI;
    double top = (double)roi.Top - (double)mOccupancyMap->CenterY;
    double buttom = (double)(roi.Top + roi.Height) - (double)mOccupancyMap->CenterY;
    double left = (double)roi.Left - (double)mOccupancyMap->CenterX;
    double right = (double)(roi.Left + roi.Width) - (double)mOccupancyMap->CenterY;
    
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, left);
    bounds.setHigh(0, right);
    bounds.setLow(1, top);
    bounds.setHigh(1, buttom);
    
    mSpace->as<ob::SE2StateSpace>()->setBounds(bounds);
    
    mProbDef->setStartAndGoalStates(start, goal);
    ob::PlannerStatus stat = mPlanner->solve(mTimeout);
    
    if( stat )
    {
        boost::shared_ptr<og::PathGeometric> sol = boost::dynamic_pointer_cast<og::PathGeometric>(mProbDef->getSolutionPath());
        const std::vector<ob::State*>& states = sol->getStates();
        mPlan.clear();
        mPlan.reserve(states.size());
        common::MotionPlanner::RobotState stat;
        for(size_t i = 0; i < states.size(); i++)
        {
             const ob::SE2StateSpace::StateType* curr_state = states[i]->as<ob::SE2StateSpace::StateType>();
             stat.Position = Eigen::Vector2f(curr_state->getX(), curr_state->getY());
             stat.Yaw = curr_state->getYaw();
             mPlan.push_back(stat);
        }
        
        return MotionPlanner::SUCCESS;
    }
    else if( stat == ob::PlannerStatus::TIMEOUT )
        return MotionPlanner::TIMEOUT;
    else
         return MotionPlanner::FAIL;       
}

bool RRTPlanner::IsValidState(const ompl::base::State* st) const
{
    const ob::SE2StateSpace::StateType* stateData = st->as<ob::SE2StateSpace::StateType>();
    // First should see the robot is in which cell, then test for cells that fall into a certain distance to the robot;
    // for all those cells, if any was occupied, then the robot cannot move to that position and therefor the state is not valid.
    // For now we do not care about robot's yaw angle.
    const size_t robot_x = stateData->getX() / mOccupancyMap->Resolution, robot_y = stateData->getY() / mOccupancyMap->Resolution;
    // First check the current position.
    if( mOccupancyMap->At(robot_y, robot_x) == common::OccupancyMap::OccupiedCell )
        return false;
    
    const size_t eff_rad = mEffectiveRadius / mOccupancyMap->Resolution;
    
    const size_t start_i = (robot_y > eff_rad ? robot_y - eff_rad : 0);
    const size_t end_i = std::min(robot_y + eff_rad + 1, mOccupancyMap->GetWidth());
    const size_t start_j = (robot_x > eff_rad ? robot_x - eff_rad : 0);
    const size_t end_j= std::min(robot_x + eff_rad + 1, mOccupancyMap->GetHeight());
    
    Eigen::Vector2f robot_pos(robot_x, robot_y);
    float squredDist = mEffectiveRadius / mOccupancyMap->Resolution;
    squredDist *= squredDist;
    
    for(size_t i = start_i; i < end_i; i++)
    {
        size_t start_index = mOccupancyMap->Index(i, 0);
        for(size_t j = start_j; j < end_j; j++)
        {
            //TODO: Check if inside radius.
            Eigen::Vector2f point(j, i); //j is x axis and i is y axis.
            if( (robot_pos - point).squaredNorm() > squredDist )
                continue; // not inside robot circle
            if( mOccupancyMap->At(start_index + j) == common::OccupancyMap::OccupiedCell )
                return false;
        }
    }
    
    return true;
}


} // end namespace interfaces
} // end namespace KSRobot

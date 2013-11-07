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

#include <common/Defenitions.h>
#include <interfaces/RRTPlanner.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/ScopedState.h>
#include <ompl/util/Console.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace KSRobot
{
namespace interfaces
{

RRTPlanner::RRTPlanner() : common::MotionPlanner()
{
    ompl::msg::noOutputHandler();
}

RRTPlanner::~RRTPlanner()
{
}

void RRTPlanner::ReadSettings(common::ProgramOptions::Ptr po)
{
    common::MotionPlanner::ReadSettings(po);
    
    mRange.Enabled = po->GetBool("Range.Enabled", true);
    mRange.Range = po->GetDouble("Range.Range", 2.0);
    
    mMultipleRuns.Enabled = po->GetBool("MultipleRuns.Enabled", true);
    mMultipleRuns.MaxIterations = po->GetInt("MultipleRuns.MaxIterations", 100);
    mMultipleRuns.MaxRuntimeMillisecs = po->GetInt("MultipleRuns.MaxRuntimeMillisecs", 50);
    
    if( !mMultipleRuns.Enabled )
    {
//        mMultipleRuns.Enabled = false;
        mMultipleRuns.MaxIterations = 1;
        mMultipleRuns.MaxRuntimeMillisecs = 1;
    }
}

void RRTPlanner::Initialize(size_t max_map_size)
{
    common::MotionPlanner::Initialize(max_map_size);

    mSpace.reset(new ob::SE2StateSpace);
    
    float mx = (float)max_map_size / 2;
    ob::RealVectorBounds bounds(2);
    bounds.setHigh(0, mx);
    bounds.setLow(0, -mx);
    bounds.setHigh(1, mx);
    bounds.setLow(1, -mx);
    mSpace->as<ob::SE2StateSpace>()->setBounds(bounds);
    
    mSI.reset(new ob::SpaceInformation(mSpace));
    mSI->setStateValidityChecker(boost::bind(&RRTPlanner::IsValidState, this, _1));
    
    mProbDef.reset(new ob::ProblemDefinition(mSI));
    
    mSimplifier.reset(new og::PathSimplifier(mSI));
    
    mPlanner.reset(new og::RRTConnect(mSI));
    mPlanner->setProblemDefinition(mProbDef);

    if( mRange.Enabled )
        ((og::RRTConnect*)mPlanner.get())->setRange(mRange.Range);
    
    mPlanner->setup();
}

void RRTPlanner::Clear()
{
    if( mPlanner.get() )
        mPlanner->clear();
}

static inline double AngDiff(double a1, double a2)
{
    double d = a1 - a2;
    if( d > M_PI / 2 )
        d -= M_PI;
    else if( d < -M_PI / 2 )
        d += M_PI;
    
    return d > 0 ? d : -d;
}

common::MotionPlanner::PlannerResult RRTPlanner::Plan()
{
    mPlan.clear();
    // extract bounds from ROI (region of interest)
    
    ob::ScopedState<ob::SE2StateSpace> start(mSpace);
    ob::ScopedState<ob::SE2StateSpace> goal(mSpace);
    
    start->setXY(mStartState.Position[0], mStartState.Position[1]);
    start->setYaw(mStartState.Yaw);
    goal->setXY(mGloalState.Position[0], mGloalState.Position[1]);
    goal->setYaw(mGloalState.Yaw);

    mProbDef->setStartAndGoalStates(start, goal);
    
    boost::shared_ptr<og::PathGeometric> best_solution;
    int iter = 0;
    int total_time;
    common::TimePoint start_time = common::Clock::now();
    
    double best_path_length = std::numeric_limits<double>::max();
    bool bSolFound = false;
    
    do
    {
        ob::PlannerStatus stat = mPlanner->solve(mTimeout);
        
        if( stat )
        {
            bSolFound = true;
            boost::shared_ptr<og::PathGeometric> sol = boost::dynamic_pointer_cast<og::PathGeometric>(mProbDef->getSolutionPath());
            
            mSimplifier->reduceVertices(*sol);
            
            const std::vector<ob::State*>& states = sol->getStates();
            double total_length = 0.0;
            for(size_t i = 1; i < states.size(); i++)
            {
                const ob::SE2StateSpace::StateType* curr_state = states[i]->as<ob::SE2StateSpace::StateType>();
                const ob::SE2StateSpace::StateType* prev_state = states[i - 1]->as<ob::SE2StateSpace::StateType>();
                //TODO: Add yaw angle to cost.
                total_length += 1 * (Eigen::Vector2d(curr_state->getX(), curr_state->getY())
                                -Eigen::Vector2d(prev_state->getX(), prev_state->getY())).squaredNorm() + 
                                0 * AngDiff(prev_state->getYaw(), curr_state->getYaw());
            }
            
            if( total_length < best_path_length )
            {
                best_path_length = total_length;
                best_solution = boost::dynamic_pointer_cast<og::PathGeometric>(sol);
            }
        }
        
        total_time = boost::chrono::duration_cast<boost::chrono::milliseconds>(common::Clock::now() - start_time).count();
        iter++;
    } while(iter < mMultipleRuns.MaxIterations && total_time < mMultipleRuns.MaxRuntimeMillisecs);
    
    if( bSolFound )
    {
        mSimplifier->reduceVertices(*best_solution);
        
        const std::vector<ob::State*>& states = best_solution->getStates();
        
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
    else if( total_time > mMultipleRuns.MaxRuntimeMillisecs )
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
    const int robot_x = stateData->getX() / mOccupancyMap->Resolution, robot_y = stateData->getY() / mOccupancyMap->Resolution;
    // First check the current position.
    if( mOccupancyMap->AtCentered(robot_y, robot_x) == common::OccupancyMap::OccupiedCell )
        return false;
    
    const int eff_rad = mEffectiveRadius / mOccupancyMap->Resolution;

    const int start_i =  std::max(robot_y - eff_rad, -(int)mOccupancyMap->GetHeight() / 2);
    const int end_i = std::min(robot_y + eff_rad + 1, (int)mOccupancyMap->GetHeight() / 2);
    const int start_j = std::max(robot_x - eff_rad, -(int)mOccupancyMap->GetWidth() / 2);
    const int end_j= std::min(robot_x + eff_rad + 1, (int)mOccupancyMap->GetWidth() / 2);
    
    Eigen::Vector2f robot_pos(robot_x, robot_y);
    float squredDist = mEffectiveRadius / mOccupancyMap->Resolution;
    squredDist *= squredDist;
    
    for(int i = start_i; i < end_i; i++)
    {
        for(int j = start_j; j < end_j; j++)
        {
            Eigen::Vector2f point(j, i); //j is x axis and i is y axis.
            if( (robot_pos - point).squaredNorm() > squredDist )
                continue; // not inside robot circle
            if( mOccupancyMap->AtCentered(i, j) == common::OccupancyMap::OccupiedCell )
                return false;
        }
    }
    
    return true;
}


} // end namespace interfaces
} // end namespace KSRobot

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

#ifndef RRTPLANNER_H
#define RRTPLANNER_H

#include <common/MotionPlanner.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/Planner.h>


namespace KSRobot
{
namespace interfaces
{

class RRTPlanner : public common::MotionPlanner
{
public:
    typedef RRTPlanner                                      this_type;
    typedef boost::shared_ptr<this_type>                    Ptr;
    typedef boost::shared_ptr<const this_type>              ConstPtr;
    
    RRTPlanner();
    virtual ~RRTPlanner();
    
    virtual void ReadSettings(common::ProgramOptions::Ptr po);
    virtual void Initialize();
    virtual common::MotionPlanner::PlannerResult  Plan();
private:
    bool IsValidState(const ompl::base::State* st) const;
private:
    ompl::base::StateSpacePtr               mSpace;
    ompl::base::SpaceInformationPtr         mSI;
    ompl::base::ProblemDefinitionPtr        mProbDef;
    ompl::base::PlannerPtr                  mPlanner;
    
    CLASS_DEF_PYEXPORT;
};

} // end namespace interfaces
} // end namespace KSRobot

#endif // RRTPLANNER_H

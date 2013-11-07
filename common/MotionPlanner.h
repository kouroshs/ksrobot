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

#ifndef MOTIONPLANNER_H
#define MOTIONPLANNER_H

#include <common/Defenitions.h>
#include <common/OccupancyMap.h>
#include <common/ProgramOptions.h>
#include <Eigen/Dense>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace KSRobot
{
namespace common
{

class MotionPlanner
{
public:
    typedef MotionPlanner                               this_type;
    typedef boost::shared_ptr<this_type>                Ptr;
    typedef boost::shared_ptr<const this_type>          ConstPtr;
    
    struct RobotState
    {
        Eigen::Vector2f                 Position;
        float                           Yaw;
        //dummy operator == for python exportation
        inline bool operator == (const RobotState& other) { return Position == other.Position && Yaw == other.Yaw; }
    };
    
    typedef std::vector<RobotState>     StateVector;

    enum PlannerResult
    {
        SUCCESS,
        TIMEOUT,
        FAIL
    };
    
    MotionPlanner();
    virtual ~MotionPlanner();
    
    inline void                                 SetStartState(const Eigen::Vector2f& position, float yaw);
    inline void                                 SetGoalState(const Eigen::Vector2f& position, float yaw);
    
    inline RobotState                           GetStartState() const;
    inline RobotState                           GetGoalState() const;
    
    inline void                                 SetTimeout(float timeInSeconds);
    inline float                                GetTimeout() const;
    
    inline void                                 SetSafeThreshold(float thr);
    inline float                                GetSafeThreshold() const;
    
    inline void                                 SetRobotRadius(float radius);
    inline float                                GetRobotRadius() const;
    
    inline StateVector                          GetPlan() const;
    
    inline void                                 SetOccupancyMap(OccupancyMap::Ptr ptr);
    inline OccupancyMap::Ptr                    GetOccupancyMap() const;
    
    virtual void                                ReadSettings(ProgramOptions::Ptr po);
    virtual void                                Initialize(size_t max_map_size);
    virtual PlannerResult                       Plan() = 0;
    virtual void                                Clear() = 0;
    
    static std::string                          GetPlannerResultString(PlannerResult res);
protected:
    float                                       mRobotRadius;
    float                                       mSafeThr;
    float                                       mTimeout;
    float                                       mEffectiveRadius;
    
    size_t                                      mMaxMapSize;
    RobotState                                  mStartState;
    RobotState                                  mGloalState;
    
    StateVector                                 mPlan;
    OccupancyMap::Ptr                           mOccupancyMap;
    CLASS_DEF_PYEXPORT;
};

inline MotionPlanner::RobotState MotionPlanner::GetGoalState() const
{
    return mGloalState;
}

inline float MotionPlanner::GetSafeThreshold() const
{
    return mSafeThr;
}

inline MotionPlanner::RobotState MotionPlanner::GetStartState() const
{
    return mStartState;
}

inline float MotionPlanner::GetTimeout() const
{
    return mTimeout;
}

inline void MotionPlanner::SetGoalState(const Eigen::Vector2f& position, float yaw)
{
    mGloalState.Position = position;
    mGloalState.Yaw = yaw;
}

inline void MotionPlanner::SetSafeThreshold(float thr)
{
    mSafeThr = thr;
}

inline void MotionPlanner::SetStartState(const Eigen::Vector2f& position, float yaw)
{
    mStartState.Position = position;
    mStartState.Yaw = yaw;
}

inline void MotionPlanner::SetTimeout(float timeInSeconds)
{
    mTimeout = timeInSeconds;
}

inline MotionPlanner::StateVector MotionPlanner::GetPlan() const
{
    return mPlan;
}

inline float MotionPlanner::GetRobotRadius() const
{
    return mRobotRadius;
}

inline void MotionPlanner::SetRobotRadius(float radius)
{
    mRobotRadius = radius;
}

inline OccupancyMap::Ptr MotionPlanner::GetOccupancyMap() const
{
    return mOccupancyMap;
}

inline void MotionPlanner::SetOccupancyMap(OccupancyMap::Ptr ptr)
{
    mOccupancyMap = ptr;
}


} // end namespace common
} // end namespace KSRobot

#endif // MOTIONPLANNER_H

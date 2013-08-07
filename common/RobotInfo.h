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

#ifndef ROBOTINFO_H
#define ROBOTINFO_H

#include <common/ProgramOptions.h>
#include <boost/shared_ptr.hpp>
namespace KSRobot
{
namespace common
{

// This class is to centeralize robot information shared with various classes. For example both VisualOdometry and MappingInterface might 
// need robot's height, and others might need it's radius. Adding a setting for each class xml file is not a practical or a good solution.
// So we combine all settings needed for the robot and use this class.
class RobotInfo
{
public:
    typedef RobotInfo                                   this_type;
    typedef boost::shared_ptr<this_type>                Ptr;
    typedef boost::shared_ptr<const this_type>          ConstPtr;
    
    RobotInfo();
    virtual ~RobotInfo(); // this way we can extend the class for special robots. I cant think of how though.
    
    inline bool                                         AddHeightToOdometry() const { return mAddHeightToOdometry; }
    inline float                                        GetConstantHeight() const { return mConstantHeight; }
    inline float                                        GetRadius() const { return mRadius; }
    
    virtual void                                        ReadSettings(ProgramOptions::Ptr po);
    
protected:
    bool                                                mAddHeightToOdometry;
    float                                               mConstantHeight;
    float                                               mRadius;
};

} // end namespace common
} // end namespace KSRobot


#endif // ROBOTINFO_H

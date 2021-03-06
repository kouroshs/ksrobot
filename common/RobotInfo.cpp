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

#include <common/RobotInfo.h>

namespace KSRobot
{
namespace common
{

RobotInfo::RobotInfo()
{
    mAddHeightToOdometry = false;
    mConstantHeight = 0.0f;
    mRadius = 0.0f;
}

RobotInfo::~RobotInfo()
{
}

void RobotInfo::ReadSettings(ProgramOptions::Ptr po)
{
    mAddHeightToOdometry = po->GetBool("AddHeightToOdometry", true);
    mConstantHeight = po->GetDouble("ConstantHeight", 0.0);
    mRadius = po->GetDouble("Radius", 0.0);
}



} // end namespace common
} // end namespace KSRobot
    
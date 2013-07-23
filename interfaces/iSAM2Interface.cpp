/*
 * <one line to give the library's name and an idea of what it does.>
 * Copyright (C) 2013  Kourosh <kourosh.sartipi@gmail.com>
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

#include <interfaces/iSAM2Interface.h>
#include <gtsam/nonlinear/ISAM2.h>


namespace KSRobot
{
namespace interfaces
{

iSAM2Interface::iSAM2Interface() : common::SLAMInterface()
{
    Initialize();
}

iSAM2Interface::~iSAM2Interface()
{
}

void iSAM2Interface::Initialize()
{
    //TODO: Change these by model
    mISAM2 = new gtsam::ISAM2();
    mPriorNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector_(1,0,0,1));
    mOdometryNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector_(1,0,0,1));
}


void iSAM2Interface::ReadSettings(common::ProgramOptions::Ptr po)
{
    //TODO:
    
    Initialize();
}

void iSAM2Interface::AddKeyframe(const common::VisualKeyframe& kf)
{
    //TODO:
}

void iSAM2Interface::AddLoopClosure(const common::LoopDetector::LoopClosure& lc)
{
    //TODO:
}


};
};

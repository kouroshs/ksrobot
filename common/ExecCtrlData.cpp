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

#include <common/ExecCtrlData.h>
#include <stdexcept>

namespace KSRobot
{
namespace common
{

void ExecCtrlData::EnableComm(bool enable)
{
//     if( !OMPL.Enable )
//         throw std::runtime_error("Cannot enable communications while OMPL is disabled.");
    Comm.Enable = enable; // for now no dependency
}

void ExecCtrlData::EnableFovis(bool enable)
{
//     if( !enable && (OctoMap.Enable || OMPL.Enable || iSAM.Enable) )
//         throw std::runtime_error("Cannot disable Fovis. Cause : OctoMap|OMPL|iSAM");
    Fovis.Enable = enable;
}

void ExecCtrlData::EnableSAM(bool enable)
{
    //NOTE: For now SAM is independent
//     if( enable && !Fovis.Enable )
//         throw std::runtime_error("Cannot enable iSAM while Fovis is disabled.");
    iSAM.Enable = enable;
}

void ExecCtrlData::EnableOctoMap(bool enable)
{
//     if( enable && !Fovis.Enable )
//         throw std::runtime_error("Cannot enable OctoMap while Fovis is disabled");
//     
//     if( !enable && OMPL.Enable )
//         throw std::runtime_error("Cannot disable OctoMap while OMPL is set.");
//     
    OctoMap.Enable = enable;
}

void ExecCtrlData::EnableOMPL(bool enable)
{
//     if( !Fovis.Enable || !OctoMap.Enable )
//         throw std::runtime_error("Cannot enable OMPLE. Cause: Fovis|OctoMap");
//     
    OMPL.Enable = enable;
}

void ExecCtrlData::CheckConsistancy() const
{
    if( !Fovis.Enable )
    {
        if( OctoMap.Enable )
            throw std::runtime_error("Cannot enable OctoMap while Fovis is disabled.");
        if( iSAM.Enable )
            throw std::runtime_error("Cannot enable iSAM while Fovis is disabled.");
        if( Comm.Enable )
            throw std::runtime_error("Cannot enable Communications while Fovis is disabled.");
    }
    
    if( !OctoMap.Enable )
    {
        if( OMPL.Enable )
            throw std::runtime_error("Cannot enable OMPL while OctoMap is disabled.");
    }

    if( !iSAM.Enable )
    {
        // For now iSAM is independent.
    }
    
    if( !OMPL.Enable )
    {
        if( Comm.Enable )
            throw std::runtime_error("Cannot enable Communications while OMPL is disabled.");
    }
}


} // end namespace utils
} // end namespace KSRobot

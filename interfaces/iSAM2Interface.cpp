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

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Symbol.h>
#include <fcntl.h>



namespace KSRobot
{
namespace interfaces
{

static inline std::string KeyToString(const gtsam::Key& k)
{
    return gtsam::Symbol(k);
}

iSAM2Interface::iSAM2Interface() : common::SLAMInterface(), mLastestCycle(0), mAddFeatures(false)
{
    // Noise model is x, y, z, roll, pitch, yaw (in meters and radians, respectively)
    mPriorNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector_(6, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1));
    mOdometryNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector_(6, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1));
    mLoopClosureNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector_(6, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1));
    
    SetInterfaceName("iSAM2Interface");
    Initialize();
}

iSAM2Interface::~iSAM2Interface()
{
}

void iSAM2Interface::Initialize()
{
    //TODO: Change these by model
    mISAM2 = new gtsam::ISAM2();
    mKeysCollection.clear();
    mNewFactors.resize(0);
    mNumFeatures = 0;
    mNumNodes = 0;
}

void iSAM2Interface::ReadSettings(common::ProgramOptions::Ptr po)
{
    common::SLAMInterface::ReadSettings(po);
    
    mPriorNoise = gtsam::noiseModel::Diagonal::Sigmas(ReadSigmas(po->StartNode("PriorNoise")));
    mOdometryNoise = gtsam::noiseModel::Diagonal::Sigmas(ReadSigmas(po->StartNode("OdometryNoise")));
    mLoopClosureNoise = gtsam::noiseModel::Diagonal::Sigmas(ReadSigmas(po->StartNode("LoopClosureNoise")));
    
    mAddFeatures = po->GetBool("UseFeatures", true);
    
    Initialize();
}

gtsam::Vector iSAM2Interface::ReadSigmas(common::ProgramOptions::Ptr po)
{
    double x, y, z, roll, pitch, yaw;
    x = po->GetDouble("X", 0.1);
    y = po->GetDouble("Y", 0.1);
    z = po->GetDouble("Z", 0.1);
    roll = po->GetDouble("Roll", 0.1);
    pitch = po->GetDouble("Pitch", 0.1);
    yaw = po->GetDouble("Yaw", 0.1);
    
    return gtsam::Vector_(6, x, y , z, roll, pitch, yaw);
}

gtsam::Pose3 Convert(const Eigen::Isometry3f& iso)
{
    gtsam::Matrix3 mat = iso.linear().cast<double>();
    Eigen::Vector3d trans = iso.translation().cast<double>();
    return gtsam::Pose3(mat, gtsam::Point3(trans));
}

void iSAM2Interface::AddKeyframe(const common::VisualKeyframe::Ptr kf)
{
//     Debug("(iSAM2Interface::AddKeyframe) New keyframe. Ref cycle = %u, Curr cycle = %u\n", kf->ReferenceCycle, kf->CurrentCycle);
    //Add reference frame.
    mSeenCycles.insert(kf->ReferenceCycle);
    mSeenCycles.insert(kf->CurrentCycle);

//     Debug("Cycle %d\n", GetCycle());
    
    Eigen::Matrix3d rot_mat = kf->RelativeMotion.linear().cast<double>();
    gtsam::Rot3 rot(rot_mat);
    gtsam::Point3 point(kf->RelativeMotion.translation().cast<double>());
    gtsam::Pose3 pose(rot, point);
    
    bool ref_found, curr_found;
    gtsam::Key ref_key = GetOrCreateNodeKey(kf->ReferenceCycle, &ref_found);
    gtsam::Key curr_key = GetOrCreateNodeKey(kf->CurrentCycle, &curr_found);
    
    if( mNumNodes - 2 == 0 ) // this is becuase we create 2 nodes just before this
    {
        mNewFactors.add(gtsam::PriorFactor<gtsam::Pose3>(ref_key, gtsam::Pose3::identity(), mPriorNoise));
        mValues.insert(ref_key, gtsam::Pose3::identity());
    }
    
    mNewFactors.add(gtsam::BetweenFactor<gtsam::Pose3>(ref_key, 
                                                       curr_key,
                                                       pose, mOdometryNoise));//between factor 
    
    //TODO: IS THIS CORRECT?
    mValues.insert(curr_key, Convert(kf->GlobalPose));
    
//     Debug("ref_key = %s, curr_key = %s\n", KeyToString(ref_key).c_str(), KeyToString(curr_key).c_str());
//     Debug("Between factors added, ref_found = %s, curr_found = %s\n", 
//           ref_found ? "true" : "false",
//           curr_found ? "true" : "false");
    
    if( mAddFeatures )
    {
        for(size_t i = 0; i < kf->MatchedPairs.size(); i++)
            GetOrCreateFeatureKey(kf->ReferenceCycle, kf->MatchedPairs[i].ReferenceIndex);
        //TODO: Complete this.
    }
}

void iSAM2Interface::AddLoopClosure(const common::LoopDetectorInterface::LoopClosure& lc)
{
    //NOTE: For loop closure, both frames should have been visited and exist inside the graph.
    if( mSeenCycles.find(lc.Cycle1) == mSeenCycles.end() || mSeenCycles.find(lc.Cycle2) == mSeenCycles.end() )
    {
        // THIS IS BAD
        Error("(iSAM2Interface::AddLoopClosure) Unknown loop closure cycles (%lu, %lu) were passed. Latest Cycle = %lu.\n", 
                  lc.Cycle1, lc.Cycle2, mLastestCycle);
        return;
    }
    
    gtsam::Key key1, key2;
    bool key1_created, key2_created;
    
    key1 = GetOrCreateNodeKey(lc.Cycle1, &key1_created);
    key2 = GetOrCreateNodeKey(lc.Cycle2, &key2_created);
    
    assert( (!key1_created) && (!key2_created) );// none of the keys should be created, they should already exist
    
    gtsam::Matrix3 mat = lc.Transform.linear().cast<double>();
    gtsam::Rot3 rot(mat);
    gtsam::Point3 trans(lc.Transform.translation().cast<double>());
    gtsam::Pose3 pose(rot, trans);
    
    mNewFactors.add(gtsam::BetweenFactor<gtsam::Pose3>(key1, key2, pose, mLoopClosureNoise));
}

void iSAM2Interface::Update()
{
//     Debug("New factors:\n");
//     mNewFactors.print();
//     Debug("New values:\n");
//     mValues.print();
//     Debug("before update\n");
    mISAM2->update(mNewFactors, mValues);
//     Debug("after update\n\n");
    //In the final act, clear new factors
    mNewFactors.resize(0);
    mValues.clear();
}

gtsam::Key iSAM2Interface::CreateFeatureKey()
{
    return gtsam::Symbol('l', mNumFeatures++);
}

gtsam::Key iSAM2Interface::CreateNodeKey()
{
    return gtsam::Symbol('x', mNumNodes++);
}

gtsam::Key iSAM2Interface::GetOrCreateNodeKey(std::size_t node_cycle, bool* was_created)
{
    bool created;
    
    gtsam::Key key;
    IndexToNodeMap::iterator iter = mNodesCollection.find(node_cycle);
    if( iter != mNodesCollection.end() )
    {
        created = false;
        key = iter->second;
    }
    else
    {
        created = true;
        key = CreateNodeKey();
        mNodesCollection[node_cycle] = key;
    }
    
    if( was_created )
        *was_created = created;
    
    return key;
}

gtsam::Key iSAM2Interface::GetOrCreateFeatureKey(std::size_t node_cycle, int feature, bool* was_created)
{
    bool created;
    gtsam::Key key;
    
    if( mKeysCollection.find(node_cycle) == mKeysCollection.end() )
        mKeysCollection[node_cycle] = IndexToKeyMap();
    
    IndexToKeyMap& i2k = mKeysCollection[node_cycle];
    IndexToKeyMap::iterator iter = i2k.find(feature);
    
    if( iter == i2k.end() )
    {
        created = true;
        key = CreateFeatureKey();
        i2k[feature] = key;
    }
    else
    {
        created = false;
        key = iter->second;
    }
    
    if( was_created )
        *was_created = created;
    
    return key;
}


};
};

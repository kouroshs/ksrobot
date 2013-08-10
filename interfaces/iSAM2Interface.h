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

#ifndef ISAM2INTERFACE_H
#define ISAM2INTERFACE_H

#include <common/SLAMInterface.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <set>
#include <map>

namespace KSRobot
{
namespace interfaces
{

class iSAM2Interface : public common::SLAMInterface
{
public:
    typedef iSAM2Interface                              this_type;
    typedef boost::shared_ptr<this_type>                Ptr;
    typedef boost::shared_ptr<const this_type>          ConstPtr;
    
    iSAM2Interface();
    virtual ~iSAM2Interface();
    
    virtual void Initialize();
    virtual void ReadSettings(common::ProgramOptions::Ptr po);
    
    void            Print() { mISAM2->print(); }
    
protected:
    virtual void                AddKeyframe(const common::VisualKeyframe::Ptr kf);
    virtual void                AddLoopClosure(const common::LoopDetector::LoopClosure& lc);
    virtual void                Update();
    
    static gtsam::Vector        ReadSigmas(common::ProgramOptions::Ptr po);
    
    gtsam::Key                  CreateNodeKey();
    gtsam::Key                  CreateFeatureKey();
    gtsam::Key                  GetOrCreateNodeKey(size_t node_cycle, bool* was_created = NULL);
    gtsam::Key                  GetOrCreateFeatureKey(size_t node_cycle, int feature, bool* was_created = NULL);
private:
    gtsam::ISAM2*                               mISAM2;
    gtsam::NonlinearFactorGraph                 mNewFactors;
    gtsam::Values                               mValues;
    
    gtsam::noiseModel::Diagonal::shared_ptr     mPriorNoise;
    gtsam::noiseModel::Diagonal::shared_ptr     mOdometryNoise;
    gtsam::noiseModel::Diagonal::shared_ptr     mLoopClosureNoise;
    
    std::set<size_t>                            mSeenCycles;
    size_t                                      mLastestCycle;
    
    typedef std::map<int, gtsam::Key>           IndexToKeyMap;
    typedef std::map<size_t, IndexToKeyMap>     IndexToKeyCollection;
    typedef std::map<size_t, gtsam::Key>        IndexToNodeMap;
    
    IndexToKeyCollection                        mKeysCollection;
    IndexToNodeMap                              mNodesCollection;
    
    size_t                                      mNumFeatures; // this is to keep track of feature numbers.
    size_t                                      mNumNodes; // this is actually number of keyframes received.
    
    bool                                        mAddFeatures;
};

};
};

#endif // ISAM2INTERFACE_H

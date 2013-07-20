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

#ifndef LOOPDETECTOR_H
#define LOOPDETECTOR_H

#include <common/Interface.h>
#include <common/VisualOdometryInterface.h>
#include <boost/signals2.hpp>
#include <tbb/concurrent_queue.h>

#include <libpmk/avt/incremental-vocabulary-tree.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>


namespace KSRobot
{
namespace common
{

// Loop detection using vocabulary trees
//TODO: Move parts to a new interface at interfaces
class LoopDetector : public Interface
{
public:
    class LoopClosure
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Eigen::Isometry3f               Transform; // Transform from 1 to 2
        int                             Cycle1;
        int                             Cycle2;
    };
    
    typedef LoopDetector                                    this_type;
    typedef boost::shared_ptr<this_type>                    Ptr;
    typedef boost::shared_ptr<const this_type>              ConstPtr;
    
    LoopDetector();
    virtual ~LoopDetector();
    
    virtual void                                            RegisterToVO(VisualOdometryInterface::Ptr vo);
    boost::signals2::connection                             RegisterLoopReceiver(boost::function<void(const LoopClosure& lc)> fn);
    
    virtual bool                                            RunSingleCycle();
    
    virtual void                                            ReadSettings(ProgramOptions::Ptr po);
protected:
    struct KeyframeData
    {
        VisualKeyframe::Ptr             Keyframe;
        KinectRgbImage::ConstPtr        Image;
        KinectRawDepthImage::ConstPtr   RawDepth;
    };
    
    void                                                    OnNewKeyframe();
    void                                                    ExtractSceneDescriptors();
    bool                                                    CheckForLoopClosure(int candidate, Eigen::Isometry3f& transform);
    void                                                    ExtractImageKeypointDescriptors(const KeyframeData& kd);
    void                                                    MatchFeatures(int candidate, std::vector<cv::DMatch>& matches);
    void                                                    Extract3DPositions(const KeyframeData& kd, const std::vector<cv::KeyPoint>& keypoints);
    cv::Mat&                                                CurrentDescriptor() { assert(mKeyframeImageKeypointDescriptors.size() != 0); return mKeyframeImageKeypointDescriptors.back(); }
    
    void                                                    ExtractValidMatches(const std::vector<cv::DMatch>& matches, int prev_cycle, int curr_cycle,
                                                                                std::vector<int>& idx_prevcycle, std::vector<int>& idx_currcycle);
    bool                                                    Ransac(int prev_cycle, int curr_cycle, const std::vector<int>& prev_idx, 
                                                                   const std::vector<int>& curr_idx, Eigen::Isometry3f& transform);
protected:
    tbb::concurrent_queue<KeyframeData>                     mKeyframesQueue;
    VisualOdometryInterface::Ptr                            mVO;
    boost::signals2::signal<void(const LoopClosure& lc)>    mOnLoopDetected;
    incremental_vtree::IncrementalVocabularyTree            mVTree;
    PointSet*                                               mCurrSceneDescriptor;
    
    int                                                     mNumCandidateImages;
    bool                                                    mUseFovisDescriptor;
    bool                                                    mNormalizeFovisDescriptor;
    std::string                                             mFeatureDetectorName;
    std::string                                             mDescriptorExtractorName;
    int                                                     mMinimumFeatureMatches;
    
    std::vector<cv::Mat>                                    mKeyframeImageKeypointDescriptors;
    std::vector<std::vector<Eigen::Vector3f> >              mPointArrayList;
    cv::Ptr<cv::FeatureDetector>                            mFeatureDetector;
    cv::Ptr<cv::DescriptorExtractor>                        mDescExtractor;
    cv::FlannBasedMatcher                                   mMatcher;
};

};
};

#endif // LOOPDETECTOR_H

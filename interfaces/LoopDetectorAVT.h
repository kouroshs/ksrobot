/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2013  <copyright holder> <email>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 * 
 */

#ifndef LOOPDETECTORAVT_H
#define LOOPDETECTORAVT_H

#include <common/LoopDetectorInterface.h>
// #include <libpmk/avt/incremental-vocabulary-tree.h>
// #include <opencv2/core/core.hpp>
// #include <opencv2/features2d/features2d.hpp>
// 
// #include <boost/random/mersenne_twister.hpp>

//NOTE: THIS CLASS IS NO LONGER SUPPORTED

namespace KSRobot
{
namespace interfaces
{

class LoopDetectorAVT : public common::LoopDetectorInterface
{
public:
    typedef LoopDetectorAVT                     this_type;
    typedef boost::shared_ptr<this_type>        Ptr;
    typedef boost::shared_ptr<const this_type>  ConstPtr;
    
    LoopDetectorAVT();
    ~LoopDetectorAVT();

protected:
    
//     virtual void                                            ReadSettings(ProgramOptions::Ptr po);

    
// protected:
//     typedef std::vector<Eigen::Vector3f>                    PointArray;
//     
//     void                                                    OnNewKeyframe();
//     void                                                    ExtractSceneDescriptors();
//     bool                                                    CheckForLoopClosure(int candidate, Eigen::Isometry3f& transform);
//     void                                                    ExtractImageKeypointDescriptors(const KeyframeData& kd);
//     void                                                    MatchFeatures(int candidate, std::vector<cv::DMatch>& matches);
//     void                                                    Extract3DPositions(const KeyframeData& kd, const std::vector<cv::KeyPoint>& keypoints);
//     cv::Mat&                                                CurrentDescriptor() { assert(mKeyframeImageKeypointDescriptors.size() != 0); return mKeyframeImageKeypointDescriptors.back(); }
//     
//     void                                                    ExtractValidMatches(const std::vector<cv::DMatch>& matches, 
//                                                                                 const PointArray& prev, const PointArray& curr,
//                                                                                 std::vector<cv::DMatch>& out);
//     bool                                                    Ransac(const PointArray& prev_points, const PointArray& curr_points,
//                                                                    const std::vector<cv::DMatch>& matches, Eigen::Isometry3f& transform);
//     
//     static bool                                             ComputeHornTransform(const PointArray& prev_points, 
//                                                                                  const PointArray& curr_points,
//                                                                                  const std::vector<cv::DMatch>& matches,
//                                                                                  const std::vector<int>& indexer,
//                                                                                  Eigen::Isometry3f& transform);
//     bool                                                    IsInlier(const Eigen::Vector3f& prev, const Eigen::Vector3f& curr,
//                                                                      const Eigen::Isometry3f& transform);
//     static float                                            ComputeError(const PointArray& prev_points, const PointArray& curr_points,
//                                                                          const std::vector<cv::DMatch>& matches,
//                                                                          const std::vector<int>& consensus_set, const Eigen::Isometry3f& trans);
// protected:
//     struct InternalKeyframeInfo
//     {
//         size_t                          Cycle;
//         Eigen::Isometry3f               Pose;
//     };
//     
//     tbb::concurrent_queue<KeyframeData>                     mKeyframesQueue;
//     VisualOdometryInterface::Ptr                            mVO;
//     incremental_vtree::IncrementalVocabularyTree            mVTree;
//     PointSet*                                               mCurrSceneDescriptor;
//     
//     size_t                                                  mNumCandidateImages;
//     bool                                                    mUseFovisDescriptor;
//     bool                                                    mNormalizeFovisDescriptor;
//     std::string                                             mFeatureDetectorName;
//     std::string                                             mDescriptorExtractorName;
//     size_t                                                  mMinimumFeatureMatches;
//     
//     //RANSAC parameters
//     size_t                                                  mRansacMinSetSize;
//     size_t                                                  mRansacMaxSteps;
//     float                                                   mRansacGoodModelPercentage;
//     size_t                                                  mRansacMinInliers;
//     float                                                   mRansacMaxAcceptableError;
//     float                                                   mRansacInlierMaxSquaredDist;
//     boost::random::mt19937                                  mRansacRNG;
//     
//     int                                                     mDelayLoopDetections;
//     float                                                   mLoopCandidatesMaxRange;
//     float                                                   mLoopCandidatesMaxYawDifference;
//     
//     std::vector<InternalKeyframeInfo>                       mInternalKeyframesBookkeeping;
//     
//     std::vector<cv::Mat>                                    mKeyframeImageKeypointDescriptors;
//     std::vector<PointArray>                                 mPointArrayList;
//     cv::Ptr<cv::FeatureDetector>                            mFeatureDetector;
//     cv::Ptr<cv::DescriptorExtractor>                        mDescExtractor;
//     cv::FlannBasedMatcher                                   mMatcher;
//     
};

} // end of namespace interfaces
} // end of namespace ksrobot

#endif // LOOPDETECTORAVT_H
